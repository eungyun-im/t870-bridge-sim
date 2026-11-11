#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_bridge_node
===================
HENES T870 High-Level Controller (HLC) bridge.
True Closed-Loop Control Variant via Robust Safety-Zone Potentiometer Feedback.
"""

import collections
import glob
import math
import struct
import time

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32, String

try:
    import serial  # pyserial
except ImportError:
    serial = None

# ---- protocol constants ----
STX = b'\x53\x54\x58'          # 'S' 'T' 'X'
ETX = b'\x0D\x0A'
MODE_MANUAL, MODE_AUTO = 0, 1
ESTOP_OFF, ESTOP_ON = 0, 1
GEAR_FORWARD, GEAR_NEUTRAL, GEAR_BACKWARD = 0, 1, 2

TX_LEN = 13
RX_LEN = 17


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class VehicleBridge:
    def __init__(self):
        # ---- params ----
        self.port = rospy.get_param("~serial_port", "/dev/ttyACM0") 
        self.port_candidates = list(rospy.get_param("~serial_port_candidates", []))
        self.baud = int(rospy.get_param("~baudrate", 115200))
        self.rate_hz = float(rospy.get_param("~control_rate", 20.0))
        self.serial_retry_period = float(rospy.get_param("~serial_retry_period", 1.0))
        self.serial_open_wait = float(rospy.get_param("~serial_open_wait", 2.5))
        self._last_serial_try = 0.0

        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        # AUTO command comes from waypoint_follower as plain std_msgs (no morai_msgs /
        # erp_driver dependency): speed is the raw 0..1000 drive command, steer is degrees
        # (negative = left, matching the LLC protocol), gear is 0=F / 1=N / 2=B.
        self.auto_speed_topic = rospy.get_param("~auto_speed_topic", "/vehicle/cmd_speed")
        self.auto_steer_topic = rospy.get_param("~auto_steer_topic", "/vehicle/cmd_steer")
        self.auto_gear_topic = rospy.get_param("~auto_gear_topic", "/vehicle/cmd_gear")

        self.ax_throttle = int(rospy.get_param("~throttle_axis", 1))
        self.throttle_mode = str(rospy.get_param("~throttle_mode", "axis"))
        self.ax_forward_trigger = int(rospy.get_param("~forward_trigger_axis", 5))
        self.ax_reverse_trigger = int(rospy.get_param("~reverse_trigger_axis", 2))
        self.ax_steer = int(rospy.get_param("~steer_axis", 2))
        self.dz_throttle = float(rospy.get_param("~throttle_deadzone", 0.08))
        self.dz_steer = float(rospy.get_param("~steer_deadzone", 0.05))
        # 2026-08-26: 이 패드는 스틱을 놓아도 조향축이 0 으로 안 돌아온다.
        # 실측에서 놓을 때마다 정확히 -0.27 로 안착했다(연속 461샘플 등).
        # 그대로 두면 중립에서 setpoint=-806 이 나가 tgt 가 pot_center(500) 대신
        # 303 이 되고, 폐루프가 바퀴를 그쪽으로 붙잡아 둔다.
        # deadzone 을 키우면(throttle 처럼) 중심 근처 분해능을 통째로 버리게 되므로
        # 중심값을 빼고 남은 구간을 다시 +-1 로 펴는 방식으로 보정한다.
        self.steer_center = clamp(
            float(rospy.get_param("~steer_axis_center", 0.0)), -0.9, 0.9)
        
        # 하드웨어 최적화 부호 설정
        self.throttle_sign = -1.0 if rospy.get_param("~invert_throttle", True) else 1.0
        self.steer_sign = 1.0 if rospy.get_param("~invert_steer", True) else -1.0

        self.btn_estop = int(rospy.get_param("~estop_button", 1))
        self.btn_estop_reset = int(rospy.get_param("~estop_reset_button", 3))
        self.btn_estop_toggle = int(rospy.get_param("~estop_toggle_button", 9))
        self.btn_mode = int(rospy.get_param("~mode_toggle_button", 7))
        self.btn_deadman = int(rospy.get_param("~deadman_button", -1))

        self.manual_max_speed = int(rospy.get_param("~manual_max_speed", 900)) 
        self.manual_max_steer_deg = float(rospy.get_param("~manual_max_steer_deg", 20.0))

        # waypoint_follower already emits a raw 0..1000 drive command, so the default scale
        # is 1.0 (pass-through). auto_max_speed still caps it for safety.
        self.auto_speed_scale = float(rospy.get_param("~auto_speed_scale", 1.0))
        self.auto_max_speed = int(rospy.get_param("~auto_max_speed", 500))
        self.auto_steer_sign = float(rospy.get_param("~auto_steer_sign", -1.0))

        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.5))
        self.auto_timeout = float(rospy.get_param("~auto_timeout", 0.5))

        # ---- drive rate limiter / direction interlock / stall guard ----
        # The LLC is a pass-through: driveControl() writes DIR then PWM back to back,
        # so whatever this node puts on the wire reaches the motor unfiltered. Both the
        # accel ramp and the zero-cross interlock have to live here, not upstream.
        self.drive_accel = float(rospy.get_param("~drive_accel_raw_per_s", 250.0))
        self.drive_decel = float(rospy.get_param("~drive_decel_raw_per_s", 400.0))
        self.gear_dwell = float(rospy.get_param("~gear_change_dwell", 0.3))
        self.gear_enc_eps = int(rospy.get_param("~gear_change_enc_eps", 2))
        self.gear_max_wait = float(rospy.get_param("~gear_change_max_wait", 3.0))
        self.stall_timeout = float(rospy.get_param("~stall_timeout", 2.0))
        self.stall_min_speed = int(rospy.get_param("~stall_min_speed_raw", 60))
        # Below the motor's breakaway PWM the command only heats the coil. Disabled
        # (0) until the threshold is measured on this vehicle.
        self.drive_min_move = int(rospy.get_param("~drive_min_move_raw", 0))
        self.enc_stale = float(rospy.get_param("~encoder_stale_timeout", 0.5))

        self.dry_log_period = float(rospy.get_param("~dry_log_period", 0.1))
        self._last_log_key = None
        self._last_log_time = rospy.Time(0)

        # ---- state ----
        # boot_mode lets the vehicle start in AUTO for joystick-less autonomous tests.
        # A connected joystick can still toggle back to MANUAL at any time (button 7).
        boot_mode = str(rospy.get_param("~boot_mode", "MANUAL")).strip().upper()
        self.mode = MODE_AUTO if boot_mode == "AUTO" else MODE_MANUAL
        self.estop = bool(rospy.get_param("~boot_estop", True))
        self.alive = 0

        self.joy = None
        self.joy_stamp = rospy.Time(0)
        self.prev_buttons = []

        # AUTO command, received as three independent std_msgs topics. Only the speed
        # topic refreshes auto_stamp: it is published every cycle by waypoint_follower and
        # is what the staleness watchdog keys on.
        self.auto_speed = 0.0
        self.auto_steer_deg = 0.0
        self.auto_gear = GEAR_NEUTRAL
        self.auto_gear_explicit = False
        self.auto_stamp = rospy.Time(0)
        self._last_manual_raw = (0.0, 0.0)

        # Drive output state. INVARIANT: _out_gear only ever changes on a cycle where
        # _out_speed is 0, so DIR never flips against a spinning motor (plugging).
        self._out_speed = 0.0
        self._out_gear = GEAR_NEUTRAL
        self._gear_zero_since = None
        self._enc = 0
        self._enc_delta = 0
        self._enc_init = False
        self._enc_stamp = rospy.Time(0)
        self._enc_seen_motion = False
        self._stall_since = None
        self._stall_latched = False

        # ---- Phase B: HLC owns the steering closed loop ----
        # Pot calibration (raw A8 counts). Fill from Phase A measurements.
        self.pot_center = float(rospy.get_param("~pot_center", 347.0))   # straight-ahead
        self.pot_left   = float(rospy.get_param("~pot_left",   100.0))   # full LEFT  (min angle)
        self.pot_right  = float(rospy.get_param("~pot_right",  750.0))   # full RIGHT (max angle)
        self.steer_cmd_max_deg = float(rospy.get_param("~steer_cmd_max_deg", 20.0))

        # PID gains: input = pot-count error, output = steer motor PWM
        self.steer_kp = float(rospy.get_param("~steer_kp", 0.8))
        self.steer_ki = float(rospy.get_param("~steer_ki", 0.0))
        self.steer_kd = float(rospy.get_param("~steer_kd", 0.0))
        self.steer_pwm_min  = int(rospy.get_param("~steer_pwm_min", 40))    # overcome static friction
        self.steer_pwm_max  = int(rospy.get_param("~steer_pwm_max", 200))   # <= Arduino STEER_PWM_MAX (220)
        self.steer_deadband = float(rospy.get_param("~steer_deadband", 12.0))  # pot counts; stop jitter
        self.steer_raw_deadband = float(rospy.get_param("~steer_raw_deadband",
                                                        self.steer_deadband))
        self.steer_pwm_slew = int(rospy.get_param("~steer_pwm_slew_per_cycle", 0))

        # Feedback filter for EMI-corrupted A8: median spike-reject -> EMA smoothing.
        # Tune from Phase A noise: bigger median window / smaller alpha = smoother but laggier.
        self.adc_median_n  = max(1, int(rospy.get_param("~adc_median_window", 5)))
        self.adc_ema_alpha = float(rospy.get_param("~adc_ema_alpha", 0.3))
        self._adc_buf = collections.deque(maxlen=self.adc_median_n)
        self.steer_adc_raw = int(round(self.pot_center))
        self.steer_adc_filt = self.pot_center   # filtered feedback the PID acts on
        self.applied_pwm_fb = 0                 # PWM the LLC actually drove (echo)

        # PID internal state
        self._steer_i = 0.0
        self._steer_prev_err = 0.0
        self._last_steer_pwm_cmd = 0

        # ---- serial ----
        self.ser = None
        self.rx_buf = bytearray()
        self._open_serial()

        # ---- publishers ----
        self.pub_enc = rospy.Publisher("/vehicle/encoder", Int32, queue_size=10)
        # PHASE A: raw, unfiltered A8 ADC hijacked into the SPEED feedback field (0..1023).
        # Plot with: rqt_plot /vehicle/steer_adc_raw  to characterize EMI noise.
        self.pub_steer_adc = rospy.Publisher("/vehicle/steer_adc_raw", Int32, queue_size=10)
        self.pub_estop = rospy.Publisher("/vehicle/estop", Bool, queue_size=1, latch=True)
        self.pub_mode = rospy.Publisher("/vehicle/mode", String, queue_size=1, latch=True)
        self.pub_status = rospy.Publisher("/vehicle/status", String, queue_size=10)
        # 2026-09-09: 실제로 LLC 로 나가는 구동 raw. MANUAL 에서는 조이스틱으로
        # 계산된 값이 어디에도 안 남아서 raw->m/s 캘리브레이션을 할 수 없었다.
        # /vehicle/cmd_speed 는 AUTO 지령이라 MANUAL 에서는 비어 있고, 램프/기어
        # 인터록/스톨가드를 거치기 전 값이라 실제 인가값과도 다르다.
        # 이건 _rate_limit() 통과 후, build_tx() 로 들어가는 바로 그 값이다.
        self.pub_applied_speed = rospy.Publisher("/vehicle/applied_speed", Int32, queue_size=10)
        self.pub_applied_gear = rospy.Publisher("/vehicle/applied_gear", Int32, queue_size=10)
        self._publish_state()

        # NOTE: the legacy auto_send/estop_send/speed_send/steer_send publishers targeted
        # the old rosserial `arduino_bridge` vehicle. This node now talks to the LLC over
        # direct serial, so they were removed.

        # ---- subscribers ----
        rospy.Subscriber(self.joy_topic, Joy, self.joy_cb, queue_size=10)
        self._sub_auto()
        rospy.Subscriber("steer_adc_raw", Int32, self.steer_adc_cb, queue_size=10)

        rospy.loginfo("[vehicle_bridge] up. boot_mode=%s estop=%d auto_topics=%s/%s/%s target_pot=%.0f",
                      "AUTO" if self.mode == MODE_AUTO else "MANUAL", int(self.estop),
                      self.auto_speed_topic, self.auto_steer_topic, self.auto_gear_topic,
                      self.pot_center)

    def _open_serial(self):
        self._last_serial_try = time.time()
        if serial is None:
            rospy.logerr_throttle(5.0, "[vehicle_bridge] pyserial is not installed!")
            return

        ports_to_try = [self.port]
        for p in self.port_candidates:
            if p not in ports_to_try:
                ports_to_try.append(p)

        import os
        for port in ports_to_try:
            try:
                if not os.path.exists(port):
                    continue
                self.ser = serial.Serial(
                    port=port,
                    baudrate=self.baud,
                    timeout=0.05,
                    write_timeout=0.05
                )
                rospy.loginfo("[vehicle_bridge] Opened serial port %s @ %d, waiting %.1f sec for boot...",
                              port, self.baud, self.serial_open_wait)
                time.sleep(self.serial_open_wait)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.port = port
                return
            except Exception as e:
                rospy.logerr_throttle(5.0, "[vehicle_bridge] Failed to open serial port %s: %s", port, e)
                self.ser = None

    def _close_serial(self):
        if self.ser is None:
            return
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def steer_adc_cb(self, msg):
        adc_raw = msg.data
        if adc_raw > 0:
            self.steer_adc_raw = self._filter_adc(adc_raw)

    def _sub_auto(self):
        """AUTO inputs from waypoint_follower. Plain std_msgs so this workspace needs
        neither morai_msgs nor erp_driver."""
        rospy.Subscriber(self.auto_speed_topic, Int32, self.auto_speed_cb, queue_size=10)
        rospy.Subscriber(self.auto_steer_topic, Float32, self.auto_steer_cb, queue_size=10)
        rospy.Subscriber(self.auto_gear_topic, Int32, self.auto_gear_cb, queue_size=10)

    def joy_cb(self, msg):
        self.joy = msg
        self.joy_stamp = rospy.Time.now()
        self._last_manual_raw = self._manual_axes(msg.axes)
        self._handle_buttons(msg)

    def auto_speed_cb(self, msg):
        # Speed drives the watchdog: waypoint_follower publishes it every control cycle.
        self.auto_speed = float(msg.data)
        self.auto_stamp = rospy.Time.now()

    def auto_steer_cb(self, msg):
        self.auto_steer_deg = float(msg.data)

    def auto_gear_cb(self, msg):
        # Explicit gear wins over the sign of speed (needed for reverse parking).
        g = int(msg.data)
        if g in (GEAR_FORWARD, GEAR_NEUTRAL, GEAR_BACKWARD):
            self.auto_gear = g
            self.auto_gear_explicit = True

    def _pressed(self, buttons, idx):
        if idx < 0 or idx >= len(buttons):
            return False
        was = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return buttons[idx] == 1 and was == 0

    def _held(self, buttons, idx):
        return 0 <= idx < len(buttons) and buttons[idx] == 1

    def _handle_buttons(self, msg):
        b = msg.buttons
        if self._pressed(b, self.btn_estop):
            self.estop = True
        if self._pressed(b, self.btn_estop_reset):
            t = self._axis(msg.axes, self.ax_throttle)
            if abs(t) <= self.dz_throttle:
                self.estop = False
        if self._pressed(b, self.btn_estop_toggle):
            if not self.estop:
                self.estop = True
            else:
                t = self._axis(msg.axes, self.ax_throttle)
                if abs(t) <= self.dz_throttle:
                    self.estop = False
        if self._pressed(b, self.btn_mode):
            self.mode = MODE_AUTO if self.mode == MODE_MANUAL else MODE_MANUAL
            self._publish_state()
        self.prev_buttons = list(b)

    def _axis(self, axes, idx, default=0.0):
        return axes[idx] if 0 <= idx < len(axes) else default

    def _apply_deadzone(self, v, dz):
        return 0.0 if abs(v) <= dz else v

    def _recenter(self, v, center):
        """축 중심 오프셋을 제거하고 양쪽 구간을 각각 +-1 로 다시 편다.
        center 쪽 절반만 좁아지는 걸 막기 위해 좌/우를 따로 정규화한다."""
        if center == 0.0:
            return v
        if v >= center:
            span = 1.0 - center
            return clamp((v - center) / span, -1.0, 1.0) if span > 1e-6 else 0.0
        span = 1.0 + center
        return clamp((v - center) / span, -1.0, 1.0) if span > 1e-6 else 0.0

    def _trigger_value(self, axes, idx):
        # Xbox-compatible triggers report -1 released and +1 fully pressed.
        raw = self._axis(axes, idx, -1.0)
        value = clamp((raw + 1.0) * 0.5, 0.0, 1.0)
        return 0.0 if value <= self.dz_throttle else value

    def _manual_axes(self, axes):
        if self.throttle_mode == "trigger_pair":
            fwd = self._trigger_value(axes, self.ax_forward_trigger)
            rev = self._trigger_value(axes, self.ax_reverse_trigger)
            t = -fwd if fwd >= rev else rev
        else:
            t = self._apply_deadzone(self._axis(axes, self.ax_throttle), self.dz_throttle)
            t *= self.throttle_sign

        s = self._apply_deadzone(
            self._recenter(self._axis(axes, self.ax_steer), self.steer_center),
            self.dz_steer)
        s *= self.steer_sign
        return t, s

    def compute_command(self, now):
        # returns (gear, speed, steer_setpoint_deg100, active)
        # active=False -> steering motor OFF (estop / stale input). steer_setpoint is deg*100.
        if self.estop:
            return GEAR_NEUTRAL, 0, 0, False

        if self.mode == MODE_MANUAL:
            if self.joy is None or (now - self.joy_stamp).to_sec() > self.joy_timeout:
                return GEAR_NEUTRAL, 0, 0, False
            g, s, st = self._manual_command()
            return g, s, st, True

        # AUTO does not require a joystick; the command topic itself is the watchdog.
        if self.auto_stamp == rospy.Time(0) or (now - self.auto_stamp).to_sec() > self.auto_timeout:
            return GEAR_NEUTRAL, 0, 0, False
        g, s, st = self._auto_command()
        return g, s, st, True

    def _manual_command(self):
        axes = self.joy.axes
        if self.btn_deadman >= 0 and not self._held(self.joy.buttons, self.btn_deadman):
            return GEAR_NEUTRAL, 0, 0

        t, s = self._manual_axes(axes)
        self._last_manual_raw = (t, s)

        if t < 0:
            gear, speed = GEAR_FORWARD, int(round(min(-t, 1.0) * self.manual_max_speed))
        elif t > 0:
            gear, speed = GEAR_BACKWARD, int(round(min(t, 1.0) * self.manual_max_speed))
        else:
            gear, speed = GEAR_NEUTRAL, 0

        if abs(s) > 0.0:
            steer_out = int(round(s * self.manual_max_steer_deg * 100.0))
        else:
            steer_out = 0

        return gear, speed, int(clamp(steer_out, -2000, 2000))

    def _auto_command(self):
        """waypoint_follower sends raw speed (0..1000, sign = direction) and steer in
        degrees. An explicit /vehicle/cmd_gear overrides the sign of speed."""
        vel = self.auto_speed

        if self.auto_gear_explicit:
            gear = self.auto_gear
        else:
            gear = GEAR_FORWARD if vel >= 0 else GEAR_BACKWARD

        if gear == GEAR_NEUTRAL:
            speed = 0
        else:
            speed = int(round(clamp(abs(vel) * self.auto_speed_scale, 0, self.auto_max_speed)))
            if speed == 0:
                gear = GEAR_NEUTRAL

        steer_deg = self.auto_steer_deg * self.auto_steer_sign
        steer = int(round(steer_deg * 100.0))
        return gear, speed, int(clamp(steer, -2000, 2000))

    # ---- drive rate limiter / direction interlock / stall guard ----
    def _enc_state(self, now):
        """(fresh, moving) from the LLC encoder feedback.

        fresh=False means the feedback went stale and we cannot tell. Callers read
        that as "not moving" for the interlock (a dead encoder degrades it to
        dwell-only instead of blocking reverse forever) and as "unknown" for the
        stall guard, which then never cuts."""
        fresh = (now - self._enc_stamp).to_sec() <= self.enc_stale
        moving = fresh and abs(self._enc_delta) > self.gear_enc_eps
        return fresh, moving

    def _reset_stall(self):
        self._stall_since = None
        self._stall_latched = False

    def _update_stall(self, now, gear_req, speed_req, moving, fresh):
        """Latch a cut when a real drive command produces no wheel motion: blocked
        wheels hold the motor at stall current until something burns.

        Keyed on the *requested* command, not the ramped output. The latch forces the
        output to 0, so keying on the output would clear itself every other cycle and
        limp-cycle the motor."""
        if moving:
            self._enc_seen_motion = True

        if (self.stall_timeout <= 0.0 or gear_req == GEAR_NEUTRAL
                or speed_req < self.stall_min_speed):
            self._reset_stall()
            return
        if moving or not fresh:
            self._stall_since = None
            return
        if self._stall_since is None:
            self._stall_since = now
            return

        held = (now - self._stall_since).to_sec()
        if held < self.stall_timeout:
            return
        rospy.logwarn_throttle(
            2.0, "[vehicle_bridge] STALL: speed=%d held %.1fs with no encoder motion (enc=%d)",
            speed_req, held, self._enc)
        # Only cut once the encoder has proven it works. If it has never moved since
        # boot it is more likely unwired than the vehicle being stuck, and cutting
        # would leave the car undrivable -- warn instead.
        if self._enc_seen_motion:
            self._stall_latched = True

    def _rate_limit(self, gear_req, speed_req, active, now):
        """Slew-limit the drive command and gate every direction change through a
        confirmed standstill. Returns the (gear, speed) actually put on the wire."""
        dt = 1.0 / self.rate_hz

        # E-STOP / stale input: safety beats smoothness, cut without ramping. PWM 0
        # free-wheels the MD20A, so an instant cut costs the driver nothing.
        if not active:
            self._out_speed = 0.0
            self._out_gear = GEAR_NEUTRAL
            self._gear_zero_since = None
            self._reset_stall()
            return GEAR_NEUTRAL, 0

        fresh, moving = self._enc_state(now)
        self._update_stall(now, gear_req, speed_req, moving, fresh)
        if self._stall_latched:
            # A stall is a fault, not a ride-quality problem. The wheels are not
            # turning, so there is no driveline shock to ramp for -- every extra
            # cycle here is just more current into a locked rotor. Cut now.
            self._out_speed = 0.0
            self._gear_zero_since = None
            return self._out_gear, 0

        target = float(speed_req)
        if gear_req == GEAR_NEUTRAL:
            target = 0.0
        elif self.drive_min_move > 0 and target < self.drive_min_move:
            # Drop to 0 rather than raise to the floor: a stop command must never be
            # turned into motion.
            target = 0.0

        # A direction change ramps the gear currently on the wire down first.
        changing_gear = (gear_req != self._out_gear)
        if changing_gear:
            target = 0.0

        if target > self._out_speed:
            self._out_speed = min(target, self._out_speed + self.drive_accel * dt)
        else:
            self._out_speed = max(target, self._out_speed - self.drive_decel * dt)

        if not changing_gear:
            self._gear_zero_since = None
        elif self._out_speed <= 0.0:
            if self._gear_zero_since is None:
                self._gear_zero_since = now
            waited = (now - self._gear_zero_since).to_sec()
            # Leaving F/B, PWM 0 only *starts* the coast-down, so dwell before even
            # looking at the encoder. Leaving NEUTRAL nothing is coasting, so the
            # encoder check alone gates it -- no dead time on a normal launch.
            dwell = 0.0 if self._out_gear == GEAR_NEUTRAL else self.gear_dwell
            if waited >= dwell and not moving:
                self._out_gear = gear_req
                self._gear_zero_since = None
            elif self.gear_max_wait > 0.0 and waited >= self.gear_max_wait:
                # Bounded release: after this long at PWM 0 the vehicle has stopped
                # whatever the encoder claims. Without it, an EMI-noisy or miswired
                # encoder reporting phantom counts would block the gear forever and
                # leave the car unable to move at all.
                rospy.logwarn_throttle(
                    2.0, "[vehicle_bridge] gear %d->%d released on timeout (%.1fs at "
                    "speed 0, encoder still claims motion dEnc=%d) -- check the encoder",
                    self._out_gear, gear_req, waited, self._enc_delta)
                self._out_gear = gear_req
                self._gear_zero_since = None

        return self._out_gear, int(round(self._out_speed))

    # ---- Phase B steering closed loop (runs on the HLC) ----
    def _filter_adc(self, raw):
        """EMI filter for the raw A8 feedback: median spike-reject -> EMA smoothing.
        Returns the median-only value so callers can use it as a de-spiked 'raw'
        for the PID's early-stop check, while steer_adc_filt keeps the EMA lag
        needed to detect target overshoot."""
        self._adc_buf.append(float(raw))
        s = sorted(self._adc_buf)
        med = s[len(s) // 2]
        a = self.adc_ema_alpha
        self.steer_adc_filt = a * med + (1.0 - a) * self.steer_adc_filt
        return med

    def _steer_target_pot(self, setpoint_deg100):
        """Map a steering setpoint (deg*100, negative=left) to a target pot count.
        Two-segment so 0 always maps to pot_center even with an asymmetric pot."""
        deg = clamp(setpoint_deg100 / 100.0, -self.steer_cmd_max_deg, self.steer_cmd_max_deg)
        if self.steer_cmd_max_deg <= 0:
            return self.pot_center
        frac = deg / self.steer_cmd_max_deg          # -1..+1
        if frac >= 0:
            return self.pot_center + (self.pot_right - self.pot_center) * frac
        return self.pot_center + (self.pot_center - self.pot_left) * frac

    def _steer_pid(self, setpoint_deg100, active):
        """PID on pot-count error -> signed steer motor PWM (-pwm_max..+pwm_max).
        +err (target above feedback) => steer RIGHT (positive PWM), matching the LLC."""
        if not active:
            self._steer_i = 0.0
            self._steer_prev_err = 0.0
            self._last_steer_pwm_cmd = 0
            return 0

        target = self._steer_target_pot(setpoint_deg100)
        raw_err = target - self.steer_adc_raw
        filt_err = target - self.steer_adc_filt

        # The filtered ADC can lag badly while the steering motor is moving. Stop as soon
        # as the raw pot reaches/crosses the target so the controller does not drive
        # through center and hit the opposite end-stop.
        if abs(raw_err) <= self.steer_raw_deadband or raw_err * filt_err < 0.0:
            self._steer_i = 0.0
            self._steer_prev_err = raw_err
            self._last_steer_pwm_cmd = 0
            return 0

        err = filt_err
        if abs(err) <= self.steer_deadband:
            self._steer_prev_err = err
            self._last_steer_pwm_cmd = 0
            return 0

        dt = 1.0 / self.rate_hz
        d = (err - self._steer_prev_err) / dt
        self._steer_prev_err = err

        if self.steer_ki > 0.0:
            self._steer_i += err * dt
            i_limit = self.steer_pwm_max / self.steer_ki        # anti-windup
            self._steer_i = clamp(self._steer_i, -i_limit, i_limit)
        else:
            self._steer_i = 0.0

        out = self.steer_kp * err + self.steer_ki * self._steer_i + self.steer_kd * d
        mag = abs(out)
        if mag < self.steer_pwm_min:
            mag = self.steer_pwm_min
        if mag > self.steer_pwm_max:
            mag = self.steer_pwm_max
        cmd = int(round(mag if out > 0 else -mag))
        if self.steer_pwm_slew > 0:
            cmd = int(clamp(cmd,
                            self._last_steer_pwm_cmd - self.steer_pwm_slew,
                            self._last_steer_pwm_cmd + self.steer_pwm_slew))
        self._last_steer_pwm_cmd = cmd
        return cmd

    def build_tx(self, gear, speed, steer):
        am = self.mode
        es = ESTOP_ON if self.estop else ESTOP_OFF
        body = struct.pack(">3sBBBHhB", STX, am, es, gear, speed, steer, self.alive)
        return body + ETX

    def parse_rx(self):
        buf = self.rx_buf
        while True:
            i = buf.find(STX)
            if i < 0:
                del buf[:max(0, len(buf) - 2)]
                return
            if len(buf) - i < RX_LEN:
                del buf[:i]
                return
            frame = bytes(buf[i:i + RX_LEN])
            if frame[-2:] != ETX:
                del buf[:i + 1]
                continue
            del buf[:i + RX_LEN]
            self._publish_feedback(frame)

    def _publish_feedback(self, frame):
        try:
            _, am, es, gear, spd, steer_raw, enc, alive = struct.unpack("<3sBBBHHiB", frame[:15])
        except struct.error:
            return

        # PHASE B: SPEED field = raw A8 (control input); STEER field = applied steer PWM (echo).
        adc_raw = spd
        self.applied_pwm_fb = steer_raw
        if adc_raw > 0:                      # 0 = glitch frame, skip
            self.steer_adc_raw = self._filter_adc(adc_raw)

        # Encoder delta feeds the direction interlock and the stall guard.
        if self._enc_init:
            self._enc_delta = enc - self._enc
        else:
            self._enc_init = True
        self._enc = enc
        self._enc_stamp = rospy.Time.now()

        self.pub_enc.publish(Int32(enc))
        self.pub_steer_adc.publish(Int32(adc_raw))
        self.pub_status.publish(String(
            "fb mode=%s estop=%d gear=%d adc_raw=%d adc_filt=%.0f appliedPWM=%d enc=%d alive=%d" %
            ("AUTO" if am == MODE_AUTO else "MANUAL", es, gear, adc_raw, self.steer_adc_filt,
             self.applied_pwm_fb, enc, alive)))

    def _publish_state(self):
        self.pub_estop.publish(Bool(self.estop))
        self.pub_mode.publish(String("AUTO" if self.mode == MODE_AUTO else "MANUAL"))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.ser is None and (time.time() - self._last_serial_try) >= self.serial_retry_period:
                self._open_serial()

            gear_req, speed_req, steer_set, active = self.compute_command(now)
            gear, speed = self._rate_limit(gear_req, speed_req, active, now)
            steer = self._steer_pid(steer_set, active)   # deg setpoint -> closed-loop motor PWM
            # 선에 나가는 값 그대로 발행. 엔코더와 짝지으면 raw->m/s 가 나온다.
            self.pub_applied_speed.publish(Int32(speed))
            self.pub_applied_gear.publish(Int32(gear))
            pkt = self.build_tx(gear, speed, steer)

            if self.ser is not None:
                try:
                    self.ser.write(pkt)
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting)
                        if chunk:
                            self.rx_buf.extend(chunk)
                    if self.rx_buf:
                        self.parse_rx()
                except Exception as e:  # noqa
                    rospy.logerr_throttle(2.0, "[vehicle_bridge] serial error: %s", e)
                    self._close_serial()

            self._dry_log(now, gear, speed, speed_req, steer_set, steer, pkt)
            self.alive = (self.alive + 1) & 0xFF
            self._publish_state()
            rate.sleep()

        self._safe_stop()

    def _dry_log(self, now, gear, speed, speed_req, steer_set, steer_pwm, pkt):
        key = (self.mode, self.estop, gear, speed, steer_set, steer_pwm)
        if key != self._last_log_key or (now - self._last_log_time).to_sec() >= self.dry_log_period:
            tag = "TX" if self.ser is not None else "DRY"
            target = self._steer_target_pot(steer_set)
            raw_err = target - self.steer_adc_raw
            filt_err = target - self.steer_adc_filt
            joy_age = (now - self.joy_stamp).to_sec() if self.joy is not None else -1.0
            joy_t, joy_s = self._last_manual_raw
            rospy.loginfo("[%s] mode=%s estop=%d joy_age=%.2f joy_thr=%.2f joy_str=%.2f gear=%d speed=%d(req=%d) dEnc=%d stall=%d set=%d->tgt=%.0f adc_raw=%d adc_filt=%.0f err_raw=%.0f err_filt=%.0f pwm=%d applied_fb=%d",
                          tag, "AUTO" if self.mode == MODE_AUTO else "MANUAL",
                          int(self.estop), joy_age, joy_t, joy_s, gear, speed,
                          speed_req, self._enc_delta, int(self._stall_latched),
                          steer_set, target, self.steer_adc_raw, self.steer_adc_filt,
                          raw_err, filt_err, steer_pwm, self.applied_pwm_fb)
            self._last_log_key = key
            self._last_log_time = now

    def _safe_stop(self):
        self.estop = True
        if self.ser is not None:
            try:
                for _ in range(5):
                    self.ser.write(self.build_tx(GEAR_NEUTRAL, 0, 0))
                    self.alive = (self.alive + 1) & 0xFF
            except Exception:  # noqa
                pass
            self._close_serial()


def main():
    rospy.init_node("vehicle_bridge")
    VehicleBridge().spin()


if __name__ == "__main__":
    main()
