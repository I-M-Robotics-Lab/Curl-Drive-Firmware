# CURL-DRIVE CLI MANUAL

---

This document is the definitive technical reference for the CURL-DRIVE CLI. It details the hierarchical command structure, parsing grammar, and provides an exhaustive breakdown of every operational flag.

## CLI ARCHITECTURE AND SYNTAX

### 1. Command Hierarchy
The interface is organized into a tree-like structure:
* `cdrv` (Root)
    * `sys` (Hardware Environment)
    * `view` (Live Telemetry)
    * `ctrl` (Motion Execution)
    * `conf` (Persistent Settings)

### 2. Grammar Rules
* **Flag Identifiers**: Use either `-` or `--` (e.g., `-raw` and `--raw` are identical).
* **Value Assignment**: Inputs are passed using the `=` operator without spaces.
    * *Example*: `cdrv ctrl set --vel=1200`
* **Persistence**: Commands under `conf set` trigger an immediate write to Flash NVM.
* **Control Modes**: Issuing a `ctrl set` command automatically switches the motor controller mode (e.g., to Velocity or Position mode).

---

## COMMAND DOCUMENTATION

#### `cdrv`: Returns the welcome banner, firmware version, and hardware revision.

#### `cdrv help`: Displays the internal help table.

#### `cdrv sys temp` : Queries the STM32G4 internal thermal sensor.
* `--raw` | `-r`: Returns the 12-bit ADC integer.
* `--kelvin` | `-k`: Returns temperature in Kelvin.
* `--fahrenheit` | `-f`: Returns temperature in Fahrenheit.
* **No Flag**: Returns temperature in Celsius.

#### `cdrv sys pwr` : Queries the DC bus voltage monitoring circuit.
* `--raw` | `-r`: Returns the raw 12-bit ADC counts from the voltage divider.
* **No Flag**: Returns the bus voltage in Volts (V).

#### `cdrv sys led` : Manual override for the status LED peripheral.
* `--on`: Sets the LED to solid ON.
* `--off`: Sets the LED to solid OFF.
* `--stop`: Disables any active blinking/toggling and turns the LED OFF.
* `--set=[0|1]`: Sets state based on value; `1` is ON, `0` is OFF.
* `--blink=[Hz]`: Toggles the LED at the specified frequency (e.g., `--blink=5.0`).

#### `cdrv view drv` : SPI communication with the TI DRV8323S gate driver.
* `--all` | `-a`: Returns a full binary dump of all Status and Control registers (11-bit strings).
* **No Flag**: Returns fault flags: `UVLO`, `CPUV`, `OCP`, and `GDF`.

#### `cdrv view enc` : Feedback from the AS5047 magnetic absolute encoder.
* `--diag` | `-d`: Returns the 12-bit diagnostic register (check for magnet errors).
* `--raw` | `-r`: Returns the 14-bit absolute position (0-16383).
* `--deg`: Returns mechanical angle in degrees (0.00-359.99).
* `--rad`: Returns mechanical angle in Radians.
* `--elec` | `-e`: Returns the current FOC electrical angle.
* **No Flag**: Returns both Mechanical Degrees and Electrical Angle.

#### `cdrv view foc` : Internal state of the Field Oriented Control algorithm.
* `--adc` | `-a`: Returns phase currents IA, IB, IC in Amperes.
* `--dq` | `-i`: Returns transformed current vectors IQ (Torque) and ID (Flux).
* `--out` | `-o`: Returns applied phase voltages VA, VB, VC (PWM Duty).
* **No Flag**: Returns IQ, ID, Mech Angle, and Elec Angle.

#### `cdrv ctrl state` : Reports control loop telemetry.
* `--vel` | `-v`: Returns current velocity (RPM), target velocity, and revolution count.
* `--pos` | `-p`: Returns current position (Deg), target position, and revolution count.
* `--trq` | `-t`: Returns target currents (TarId/TarIq) and output voltages (Vd/Vq).
* **No Flag**: Returns Mode, Torque Type, Armed status, and Calibration flag.

#### `cdrv ctrl set` : Changes the operational mode and updates setpoints.
* `--trq=[val]` | `-t` | `-torque` : Sets target current mA and enters Torque Mode.
* `--vel=[val]` | `-v` | `-velocity` : Sets target RPM and enters Velocity Mode.
* `--pos=[val]` | `-p` | `-position` : Sets target Degrees and enters Position Mode.
* `--vel=[val]` | `-v=[val]`: Sets target RPM and enters Velocity Mode.
* `--pos=[val]` | `-p=[val]`: Sets target Degrees and enters Position Mode.
* `--Vd=[val]`: Injects direct-axis voltage (Open-loop).
* `--Vq=[val]`: Injects quadrature-axis voltage (Open-loop).

#### `cdrv ctrl idle` : Immediately disables the inverter and sets the controller mode to IDLE.

#### `cdrv ctrl arm` : Checks driver health, clears High-Z, and enables motor.

#### `cdrv ctrl disarm` : Forces High-Z state and resets all control variables.

#### `cdrv ctrl zero` : Capture current position as volatile 0.0 degree origin.

#### `cdrv ctrl cal` : Initiates the automated motor calibration and alignment sequence.

#### `cdrv conf get` : Retrieves settings currently active in RAM.
* `--pid`: Displays all Kp, Ki, Kd gains for every loop.
* `--lim`: Displays VMAX, IMAX, and VELMAX safety limits.
* `--hw`: Displays Pole Pairs, Shunt Resistance, ADC Gain, and Direction.
* `--freq`: Displays Current loop and Velocity loop execution frequencies.
* `--all`: Full dump of the entire configuration structure.
* `--ikp`: Returns Current loop Proportional gain.
* `--iki`: Returns Current loop Integral gain.
* `--vkp`: Returns Velocity loop Proportional gain.
* `--vki`: Returns Velocity loop Integral gain.
* `--pkp`: Returns Position loop Proportional gain.
* `--pki`: Returns Position loop Integral gain.
* `--pkd`: Returns Position loop Derivative gain.
* `--valpha`: Returns Velocity LPF alpha coefficient.
* `--palpha`: Returns Position LPF alpha coefficient.
* `--vmax`: Returns Bus overvoltage threshold (V).
* `--imax`: Returns Phase overcurrent threshold (A).
* `--velmax`: Returns Velocity limit (RPM).
* `--rsh`: Returns Shunt resistor value (Ohms).
* `--adcg`: Returns Current sense amplifier gain.
* `--pp`: Returns Motor pole pair count.
* `--sgn`: Returns Phase direction multiplier (1 or -1).
* `--eoff`: Returns 14-bit Electrical Offset.
* `--cfreq`: Returns Current loop ISR frequency (Hz).
* `--vfreq`: Returns Velocity loop ISR frequency (Hz).

#### `cdrv conf set` : Modifies settings and saves them to Flash.
* `--ikp=[val]`: Sets Current loop Proportional gain.
* `--iki=[val]`: Sets Current loop Integral gain.
* `--vkp=[val]`: Sets Velocity loop Proportional gain.
* `--vki=[val]`: Sets Velocity loop Integral gain.
* `--pkp=[val]`: Sets Position loop Proportional gain.
* `--pki=[val]`: Sets Position loop Integral gain.
* `--pkd=[val]`: Sets Position loop Derivative gain.
* `--valpha=[val]`: Sets Velocity filter alpha (0.0 to 1.0).
* `--palpha=[val]`: Sets Position filter alpha (0.0 to 1.0).
* `--vmax=[val]`: Sets Maximum Bus Voltage safety limit.
* `--imax=[val]`: Sets Maximum Phase Current safety limit.
* `--velmax=[val]`: Sets Maximum Velocity limit.
* `--rsh=[val]`: Sets Shunt Resistance for current sensing.
* `--adcg=[val]`: Sets ADC Gain for current sensing.
* `--pp=[val]`: Sets Motor Pole Pairs.
* `--sgn=[val]`: Sets Phase direction (1 or -1).
* `--eoff=[val]`: Sets 14-bit Electrical Offset.
* `--cfreq=[val]`: Sets Current loop frequency and re-initializes timer.
* `--vfreq=[val]`: Sets Velocity loop frequency and re-initializes timer.

---