# PAROL6 Commander firmware for MKS TinyBee V1

This is a clean ESP32 port of the PAROL6 control-board firmware for a custom
six-axis stepper robot:

- J1, J3, J4, J5 and J6 use the TinyBee X, Y, Z, E0 and E1 sockets.
- J2 uses an external STEP/DIR/ENABLE driver from the EXP1 header.
- The original 3 Mbaud PAROL Commander binary protocol is preserved.
- Commands 100, 101, 102, 103, 123, 156 and 255 are implemented.
- The Home command does not move the arm. It assumes the arm is parked in the
  PAROL6 standby pose (J1 +90°, J2 -90°, J3 180°, J4 0°, J5 0°, J6 180°) and
  reports that pose, with every joint homed, to PAROL Commander.
- Acceleration limiting, software travel limits, E-stop handling and
  communication timeout handling are included.

The onboard stepper signals are **not normal ESP32 GPIOs**. The TinyBee sends
them through cascaded shift registers using an I2S stream. This project includes
the required DMA-backed I2S motion engine; `AccelStepper` cannot directly drive
TinyBee pins 128-142.

## Read this before powering motors

The values in `include/robot_config.h` are safe placeholders. Do not zero or jog
the physical arm until all six `JointConfig` rows have been replaced with the
arm's real values and every direction has been tested at low speed.

Open `include/commander_units.h` and set `PHYSICAL_STEPS_PER_DEGREE` for each
joint. PAROL Commander assumes PAROL6 gearing (200 x 32 microsteps and ratios
6.4, 20, 18.095, 4, 4, 10) for everything on the serial link; the firmware
converts between those "Commander steps" and your real motor steps. The
defaults give a scale of 1.0 (no conversion).

For each joint set:

```text
steps_per_degree = motor_full_steps_per_rev × microsteps × reduction / 360
minimum_steps    = minimum_joint_angle_degrees × steps_per_degree
maximum_steps    = maximum_joint_angle_degrees × steps_per_degree
```

Angles use Commander's convention, so the parked standby pose is not zero. The
PAROL6 reference travel is approximately:

| Joint | Standby | Min | Max |
|---|---|---|---|
| J1 | +90° | -123° | +123° |
| J2 | -90° | -145° | -3.4° |
| J3 | 180° | 108° | 288° |
| J4 | 0° | -105° | +105° |
| J5 | 0° | -90° | +90° |
| J6 | 180° | 0° | 360° |

Set `direction_inverted`, travel limits and maximum speed/acceleration. The
remaining homing fields are currently unused because physical homing is bypassed.
Limits are only enforced after Home has been pressed.

The firmware deliberately starts with every driver disabled. PAROL Commander
must send command 101 before motion is allowed.

The GPIO E-stop input is a software stop, not a safety-rated emergency-stop
circuit. A physical E-stop should remove motor power through appropriately rated
hardware independently of this firmware.

## Default wiring

| Robot joint | TinyBee connection | Signal source |
|---|---|---|
| J1 | X driver socket | Onboard I2S output |
| J2 STEP | EXP1 GPIO17 | External NEMA 23 driver |
| J2 DIR | EXP1 GPIO16 | External NEMA 23 driver |
| J2 ENABLE | EXP1 GPIO21 | External NEMA 23 driver |
| J3 | Y driver socket | Onboard I2S output |
| J4 | Z driver socket | Onboard I2S output |
| J5 | E0 driver socket | Onboard I2S output |
| J6 | E1 driver socket | Onboard I2S output |
| J1-J3 home | X-STOP, Y-STOP, Z-STOP | GPIO33, 32, 22 |
| J4 home | MT-DET | GPIO35 |
| J5-J6 home | TH2, TB | GPIO34, 39 |
| E-stop | EXP1 BTN_ENC | GPIO13 |

GPIO34, GPIO35 and GPIO39 are input-only and have no internal pull-ups. Add a
4.7-10 kOhm pull-up to **3.3 V**, or use a sensor with a defined 3.3 V logic
output. Never apply 5 V to an ESP32 input.

For an optoisolated NEMA 23 driver, verify the input-current requirement. If its
PUL/DIR/ENA inputs do not reliably recognise 3.3 V, use a 3.3 V-compatible line
driver or transistor interface. Join signal grounds unless the selected wiring
scheme and driver documentation explicitly require isolated/common-anode wiring.

## Build and upload

1. Install VS Code, PlatformIO, or PlatformIO Core.
2. Open this folder as a PlatformIO project.
3. Edit `include/robot_config.h`.
4. Build with `pio run`.
5. Connect the TinyBee by USB and upload with `pio run -t upload`.

The project deliberately pins `espressif32@2.1.0`, matching the Arduino/ESP32
generation used by Makerbase's TinyBee Marlin firmware and its low-level I2S
register interface.

## First-power procedure

1. Leave motor power disconnected and upload the firmware.
2. Check that the external driver's STEP/DIR/ENABLE interface is 3.3 V safe.
3. Mechanically decouple or unload one joint where practical.
4. Set very low speed and acceleration values.
5. Connect only J1, send enable, and issue a tiny joint jog.
6. Verify motor direction, then place the complete arm in its defined zero pose.
7. Park the arm in the PAROL6 standby pose and press Home. The arm will not
   move; the firmware reports the standby pose as its current position.
8. Repeat movement tests one joint at a time and test the E-stop before enabling
   all axes.

Do not initially use Cartesian jogging: until the desktop robot model is changed,
PAROL Commander's inverse kinematics still describe the PAROL6 geometry. Joint
jogging and joint-space commands can be commissioned first.

## Protocol compatibility

Incoming frames retain the upstream layout:

```text
FF FF FF | length=52 | 6×signed-24 position | 6×signed-24 velocity |
command | affected joint | I/O | timeout | gripper fields | CRC placeholder |
01 02
```

Outgoing frames retain the upstream length of 56 bytes. Position and speed are
reported in Commander steps and steps/s (converted from your motor steps, see
`commander_units.h`). The command-interval field is in 1.4222 µs ticks, as
Commander expects. The gripper fields carry the upstream idle defaults because
the STM32 CAN gripper interface is not present on TinyBee. Incoming frames whose
length byte is not 52 are discarded.

## Serial link to a Raspberry Pi (optional)

Set `COMMANDER_ON_UART2` in `include/commander_port.h`. The firmware then talks to
Commander on UART2 instead of USB:

| Raspberry Pi | TinyBee |
|---|---|
| GPIO14 TXD (header pin 8) | GPIO14 = EXP2 pin 3 (ESP32 RX) |
| GPIO15 RXD (header pin 10) | GPIO19 = EXP2 pin 1 (ESP32 TX) |
| GND (pin 6) | GND = EXP2 pin 9 |

Do not connect 3.3 V or 5 V between the boards. Both sides are 3.3 V logic. Remove
any SD card from the TinyBee (GPIO19 is the shared SPI MISO line). The ESP32's
default UART2 pins, GPIO16/17, cannot be used: on the TinyBee they are on EXP1,
level-shifted to 5 V and output-only. This also means the J2 STEP/DIR/ENABLE
signals (GPIO17/16/21) are 5 V logic, not 3.3 V.

On the Pi, free the PL011 UART (the mini-UART cannot hold 3 Mbaud): add
`enable_uart=1` and `dtoverlay=disable-bt` to `/boot/firmware/config.txt`, turn off
the serial login console (`raspi-config`, Interface Options, Serial Port: login
shell No, hardware serial Yes), add your user to `dialout`, reboot, and use
`/dev/serial0` (= `ttyAMA0`) as the Commander's `com_port`.

## Enable, disable, timeout and E-stop

- Boot: drivers disabled. The Home command (100) enables them, because the
  Commander ("pilot") only ever sends Enable (101) to recover from an E-stop.
  Auto-home on Commander start-up therefore energises the arm; make sure it is
  parked in the standby pose first.
- Home reports the joints un-homed for about 0.1 s and then homed, because the
  Commander's Home command waits to see an un-homed state first (otherwise it
  stalls for 20 s before giving up).
- Disable (102) and communication timeout: motion stops and motion commands are
  ignored, but the drivers stay energised so the arm keeps holding (as upstream
  does). Enable (101) resumes. Upstream never enforced the timeout byte; this port
  does when it is non-zero.
- GPIO E-stop (only if `ESTOP_ENABLED`): drivers are disabled, joints are marked
  un-homed, and Enable (101) plus a fresh Home are required. Telemetry reports
  the E-stop bit as 1 when healthy. If E-stop only cuts motor power in hardware,
  leave `ESTOP_ENABLED` off; the bit is then always 1.
- Because the arm is open loop, any power cut can let it move. Park it in the
  standby pose and press Home again afterwards.

The upstream firmware also used a placeholder CRC rather than validating it;
this port preserves that behaviour for compatibility.

## Source layout

- `src/protocol.cpp` — Commander packet parser and telemetry encoder
- `src/tinybee_i2s.cpp` — TinyBee shift-register DMA output
- `src/motion.cpp` — six-axis pulse generation and acceleration limiting
- `src/homing.cpp` — retained homing state machine; currently bypassed by the
  Home-command handler in `controller.cpp`
- `src/controller.cpp` — command dispatch, I/O and safety handling
- `include/robot_config.h` — all arm-specific values and accessible pins
- `include/commander_units.h` — Commander-step ↔ motor-step scaling and the standby pose

## Current limitations

- This is an open-loop stepper controller; position feedback is inferred from
  emitted pulses.
- The PAROL6 CAN gripper and TMC5160 SPI diagnostics are intentionally omitted.
- J2 is timer-driven separately from the five I2S axes. It shares the same
  trajectory command stream but is not hardware-clock-synchronised to the I2S
  shift-register sample clock.
- Cartesian motion requires updating PAROL Commander with the custom arm's link
  geometry, joint zero conventions and joint limits.

## Licence and attribution

GPL-3.0. The protocol behaviour is based on Source Robotics' PAROL6 control-board
firmware. The TinyBee I2S output implementation is adapted from Marlin's ESP32
I2S stepper stream. See `NOTICE.md` and `LICENSE`.
