# T870 첫 세팅 가이드 (Arduino LLC ↔ PC HLC)

> ⚠️ **안전 1순위**: 모든 첫 테스트는 **차량 바퀴를 땅에서 띄운 상태(거치대/받침)**로,
> **사람이 E-Stop을 즉시 누를 수 있는 상태**에서 한다. 속도 상한은 낮게 시작.

---

## 0. 연결 방식 선택

펌웨어는 **HLC 링크 = `Serial2`(핀 16/17, RS232)**, **디버그 = USB(`Serial`)** 로 되어 있다.

- **방법 A (PDF 정석, RS232)**: Mega Serial2 → MAX3232 → USB-RS232 어댑터 → PC.
  PC에서 그 어댑터가 `serial_port`(예: `/dev/ttyUSB1`).
- **방법 B (벤치 간편, USB)**: RS232 부품이 없으면, `.ino`에서 `#define HLC Serial2`를
  `#define HLC Serial` 로 바꾸고 Arduino USB 한 가닥으로 통신. 이때 `serial_port`는
  Arduino USB(`/dev/ttyACM0` 보통). 단, 이러면 USB 디버그 출력은 못 씀.

---

## 1. 펌웨어 업로드

1. Arduino IDE에서 `arduino/t870_llc/t870_llc.ino` 열기
2. 보드 = **Arduino Mega 2560**, 포트 선택 → **Upload**
3. 업로드 직후엔 ROS 노드가 그 포트를 잡고 있으면 안 됨(껐다가)

CLI라면:
```bash
arduino-cli compile --fqbn arduino:avr:mega arduino/t870_llc
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino/t870_llc
```

## 2. 시리얼 포트 찾기 + 권한

```bash
ls /dev/ttyUSB* /dev/ttyACM*      # 꽂기 전/후 비교해서 새로 생긴 게 LLC
# IMU(iahrs)가 ttyUSB0를 쓰므로 LLC는 보통 ttyUSB1 또는 ttyACM0
sudo chmod 777 /dev/ttyUSB1       # 컨테이너엔 'tty' 별칭으로도 가능
```
찾은 포트를 `config/vehicle_bridge.yaml` 의 `serial_port` 에 기입 (또는 launch 인자로).

## 3. 통신 단독 확인 (모터 배선 전 권장)

먼저 펌웨어가 17B 피드백을 잘 보내는지만 확인:
```bash
sb
roslaunch vehicle_bridge vehicle_bridge.launch serial_port:=/dev/ttyUSB1
# 다른 터미널
rostopic echo /vehicle/status      # fb ... enc=... 가 20Hz로 갱신되면 링크 정상
rostopic echo /vehicle/encoder
```
- `cannot open ...` → 포트/권한 문제 (2번 다시)
- 피드백 안 옴 → RX/TX 교차 배선, baud(115200), GND 공통 확인

## 4. 조향 POT 캘리브레이션 (모터 연결 후)

조향 모터 배선 후, `.ino` 상수를 실제 값으로 채운다.
1. 바퀴를 **손으로** 또는 천천히 좌/중앙/우 끝으로 돌리며 `analogRead(A8)` 값 확인
   (USB 디버그 또는 `/vehicle/status`의 `steer_raw`)
2. `.ino` 의 다음을 측정값으로:
   - `POT_FULL_LEFT`  (완전 좌, 음수 조향)
   - `POT_CENTER`     (직진)
   - `POT_FULL_RIGHT` (완전 우)
3. 조향이 **반대로** 가면 `STEER_POS_LEVEL` 뒤집기
4. 떨림/오버슈트 → `STEER_KP`, `STEER_PWM_MIN/MAX`, `STEER_DEADBAND` 튜닝

## 5. 구동 방향 + 속도 상한

1. `DRIVE_PWM_MAX` 를 낮게(예: **120**) 두고 시작
2. 조이스틱 살짝 전진 → 바퀴가 **전진** 방향인지 확인. 반대면 `DRIVE_FWD_LEVEL` 뒤집기
3. 후진도 확인. 문제없으면 상한을 점진적으로 올림

## 6. 안전장치 점검 (주행 전 필수)

- **E-Stop**: 버튼 9(토글) 또는 1(걸기) → 즉시 `speed=0`, 바퀴 멈춤 확인
- **워치독**: HLC 노드를 끄거나 USB 뽑으면 300ms 내 LLC가 모터 정지하는지 확인
- **부팅 안전**: 전원만 켰을 때(HLC 없음) 모터가 안 도는지 확인 (펌웨어 부팅 ESTOP)

## 7. 정상 주행

```bash
roslaunch vehicle_bridge vehicle_bridge.launch serial_port:=/dev/ttyUSB1
# (E-Stop ON으로 시작 → 버튼으로 해제 후 주행)
```

---

### 조작 요약
| 입력 | 동작 |
|--|--|
| axis 1 | 쓰로틀 (전/후진, invert 적용) |
| axis 2 | 조향 (좌/우, invert 적용) |
| 버튼 9 | E-Stop 토글 ON/OFF |
| 버튼 1 / 3 | E-Stop 걸기 / 해제 |
| 버튼 7 | Manual ↔ Auto |
