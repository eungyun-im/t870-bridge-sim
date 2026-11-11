<div align="center">

# t870-bridge-sim

**Vehicle bridge and a lightweight simulator for a HENES T870 ride-on car,<br>used by KUUVe at the HL FMA 2026 Autonomous Driving Competition.**

![ROS](https://img.shields.io/badge/ROS_1-22314E?style=flat-square&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino_Mega-00878F?style=flat-square&logo=arduino&logoColor=white)
![RS232](https://img.shields.io/badge/RS232-115200_bps-555555?style=flat-square)

HL FMA (Future Mobility Award) 2026 Autonomous Driving Competition · Encouragement Award · Team KUUVe

[Overview](#overview) · [Vehicle bridge](#vehicle_bridge) · [Simulator](#sim_vehicle) · [Simulator vs real car](#checking-the-simulator-against-the-real-car) · [Limitations](#limitations)

</div>

---

## Overview

Two ROS 1 packages for one car:

| Package | Role |
|---|---|
| [`vehicle_bridge`](vehicle_bridge) | PC (high-level controller) ↔ Arduino Mega (low-level controller) over RS232, plus the Arduino firmware |
| [`sim_vehicle`](sim_vehicle) | Hardware-free stand-in for the car, GPS and EKF, scenario launches, and an offline simulator for parameter sweeps |

The path planner and path follower run on top of these and are not part of this repository.

## vehicle_bridge

**Protocol** (20 Hz, 115200 bps)

| Direction | Size | Fields |
|---|---|---|
| PC → Arduino | 13 B, big-endian | `STX` · auto/manual · E-stop · gear · speed · steer · alive · `CR LF` |
| Arduino → PC | 17 B, little-endian | `STX` · auto/manual · E-stop · gear · steering ADC · applied steer PWM · encoder · alive · `CR LF` |

The steering loop runs on the PC: the Arduino only drives the motor and reports the potentiometer. The potentiometer signal picked up motor noise, so the PC filters it (median window, then EMA) before the PID. All steering gains are ROS parameters, so tuning needs no reflash.

**Safety layers**

| Layer | Condition | Reaction |
|---|---|---|
| Arduino | Power on | Starts in E-stop until the PC clears it |
| Arduino | No valid packet for 300 ms | Motors stop, feedback reports E-stop |
| Arduino | Steering at the left or right end stop | Refuses to push further |
| Arduino | Any steer command | PWM capped at 220 |
| PC | Joystick or auto command older than 0.5 s | Steering motor off, no drive |
| PC | Encoder feedback older than 0.5 s | Treated as unknown, not as stopped |
| PC | Drive commanded but no encoder motion for 2 s | Drive cut and latched (only once the encoder has been seen working) |

[`FIRST_SETUP.md`](vehicle_bridge/FIRST_SETUP.md) is the bring-up checklist: wheels off the ground, E-stop in reach, watchdog and boot E-stop checked before the first drive.

## sim_vehicle

| Launch | Scenario |
|---|---|
| `sim.launch`, `sim_vehicle.launch` | Path following on a recorded course with a kinematic bicycle model |
| `sim_avoid_test.launch` | Obstacle avoidance with a fake lidar and the real avoidance node |
| `sim_slot_test.launch` | Picking the empty parking slot (T and parallel) |
| `sim_stop_test.launch` | Stopping for an obstacle on the path |
| `sim_traffic_light_test.launch` | Stop and go at the four traffic lights |
| `sim_full_test.launch` | Avoidance, both parkings and stops in one run |
| `sim_vehicle_drift.launch` | Replays failure modes recorded on the car (steering offset, GPS dropout) |
| `sim_vehicle_control_ab.launch` | A/B comparison of controller changes under the drift profile |

Every run writes a timestamped folder with a time series and a summary (`eval_metrics.py`): mission success, cross-track error per mission state, steering reversals. Runs no longer overwrite each other, so A/B comparisons are possible.

`offline_sim.py` runs the same loop without ROS for sweeps (lookahead, speed per state). `offline_validate.py` checks that it reproduces the ROS runs on the same measurement window.

## Checking the simulator against the real car

The first offline simulator predicted a maximum cross-track error of 0.42 m. The real car (run 3) reached 2.84 m. Two causes were found from recorded bags:

| Cause | Evidence | Fix |
|---|---|---|
| Wrong speed scale | Encoder calibrated against GPS: raw 350 → 1.09 m/s, raw 900 → 2.78 m/s | `v = raw × 0.003081` |
| Perfect position fed to the controller | 7 real runs: EKF lag 20 to 160 ms up to 1.86 m/s, 380 to 420 ms from 2.31 m/s | Speed-dependent lag model (logistic fit, RMS residual 44.7 ms) |

The lag fit is used for relative comparisons (which settings cross the 2.27 m/s threshold), not for absolute error predictions.

## Running

```bash
# real car
roslaunch vehicle_bridge vehicle_bridge.launch serial_port:=/dev/ttyUSB1

# simulator (needs the team's path loader and follower packages)
roslaunch sim_vehicle sim_full_test.launch
```

## Limitations

- `sim_vehicle` depends on team packages not included here (`waypoint_maker`, `waypoint_follower`, `obstacle_detector`, `obstacle_bridge`).
- The lag model rests on 6 data points, two of them taken with a different smoothing setting.
- Steering response in the simulator is not calibrated; few bags recorded the potentiometer.
- The EKF noise change made after the lag analysis was not re-verified on the car.
- Some scripts contain absolute paths from the team machine.

## License and authorship

- License: not chosen yet.
- Team KUUVe, Kookmin University. AI tools (Claude Code) were used for implementation and analysis.
