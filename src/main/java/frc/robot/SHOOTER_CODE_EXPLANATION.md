# FRC 2026 Shooter Robot Code Explanation (Current Repo)

## Project Overview
This is a command-based WPILib robot program that controls a flywheel shooter. It uses:
- Subsystems for hardware ownership.
- Commands for operator actions.
- REVLib 2026 Spark MAX configuration + closed-loop control.
- Debouncing for the limit switch.

## Architecture (Command-Based Model)

### RobotContainer (Wiring + Bindings)
- File: `src/main/java/frc/robot/RobotContainer.java`
- Role: Instantiates subsystems and binds controller buttons to commands.
- No periodic logic lives here; it only declares button -> command bindings.
- Operator Xbox and keypad bindings are enabled by `Constants.OI.kEnableOperatorXbox` and `Constants.OI.kEnableKeypadBindings`.

### ShooterSubsystem (Hardware Ownership)
- File: `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- Owns three Spark MAX controllers (leader, follower, feed) and a limit switch.
- Exposes methods used by commands: `setTargetRpm()`, `setTargetVoltage()`, `setTargetByDistance()`,
  `isLimitSwitchPressed()`, `runFeedVoltage()`, `stop()`, `stopFeed()`, `isAtSpeed()`.
- Publishes telemetry and logs values.

### Commands (Operator Actions)
- Files: `src/main/java/frc/robot/commands/`
- `SetShooterRpm`: One-shot command that sets a target RPM and ends immediately.
- `StopShooter`: One-shot command that stops the shooter.
- `RunShooterUntilStopped`: Continuous command that sets RPM on `initialize()` and stops in `end()`.
- `SetShooterRpmByDistance`: One-shot command that selects RPM or voltage based on distance.
- `RunFeederManual`: Runs feeder while scheduled; stops on end.
- `RunFeederWhenAtSpeed`: Runs feeder only when `isAtSpeed()` is true.
- `RunShooterUntilLimitSwitch`: Runs shooter until limit switch is pressed (available but not bound by default).

## Button Bindings (Exact Current Mapping)

### Operator Xbox (USB port 1 by default)
- A: `SetShooterRpm(kLowRpm)`
- B: `SetShooterRpm(kMidRpm)`
- Y: `RunShooterUntilStopped(kHighRpm)`
- X: `StopShooter()`
- Left Bumper (hold): `RunFeederManual(kFeedVoltage)`
- Right Bumper (hold): `RunFeederWhenAtSpeed(kFeedVoltage)`

Distance presets (nearest lookup, RPM mode):
- D-pad Up: Far preset
- D-pad Left: Mid preset
- D-pad Right: Near preset

Distance presets (nearest lookup, Voltage mode):
- Start: Far preset
- Back: Mid preset
- Right Stick: Near preset

Distance presets (interpolated lookup, RPM mode):
- D-pad Down: Mid preset
- Left Trigger: Near preset
- Right Trigger: Far preset

### Keypad (USB port 1 by default)
Keypad bindings are defined in `configureKeypadBindings()` and are enabled when
`Constants.OI.kEnableKeypadBindings` is true. The current mapping is a testing layout
that exposes multiple shooting algorithms for comparison. It also documents a
planned competition layout (see comments in `RobotContainer.java`).

## ShooterSubsystem Details (Current Implementation)

### Motor Setup
- Leader Spark MAX: CAN ID 27 (`Constants.Shooter.kLeaderId`)
- Follower Spark MAX: CAN ID 23 (`Constants.Shooter.kFollowerId`)
- Feed Spark MAX: CAN ID 28 (`Constants.Shooter.kFeedId`)
- Follower is configured to follow the leader with `kFollowerInverted = true`.

### Configuration Flow
1. A shared `SparkMaxConfig` `baseConfig` is built with inversion, current limit, ramp rates,
   closed-loop PID gains, I-zone, output range, and feedforward `kV` (volts per RPM).
2. Leader config applies `baseConfig` and overrides inversion to `false`.
3. `leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)`
4. `follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)`
5. `feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)`

`PersistMode.kNoPersistParameters` means settings are not flashed/persisted across power cycles.

### Loop Running Modes
Closed-loop control:
- `setTargetRpm()` clamps RPM to `0..kMaxRpm_Motor` then calls:
  `pid.setSetpoint(targetRpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0)`

Open-loop control:
- `setTargetVoltage()` clamps voltage to `0..kMaxVoltage` and calls `leader.setVoltage(...)`.

Distance-based setpoint selection:
- `setTargetByDistance()` selects either RPM or voltage from lookup tables.
- Lookup mode is selectable:
  - `LookupMode.NEAREST` (default)
  - `LookupMode.INTERPOLATED`

### Test Hardware
Limit switch:
- Uses a `DigitalInput` on DIO 0 (`Constants.Shooter.kLimitSwitchPort`).
- A `Debouncer` (0.02s) filters bounce before reporting a press.

### Telemetry and Logging
- SmartDashboard updates every 20 ms.
- DataLog entries for leader RPM and follower RPM are appended every cycle.
- Target RPM and target voltage are logged only when they change (to reduce log spam).
- "Shooter Ready" indicates when RPM is within tolerance.

## Key Constants (From `Constants.java`)

| Constant | Value | Notes |
|---|---|---|
| `kLowRpm` | 2000 | RPM |
| `kMidRpm` | 3250 | RPM |
| `kHighRpm` | 5700 | RPM |
| `kVeryLowRpm` | 1500 | RPM |
| `kVeryHighRpm` | 6200 | RPM (clamped) |
| `kMaxRpm_Motor` | profile-based | NEO/Vortex selection |
| `kP` | 0.00018 | PID |
| `kI` | 0.0 | PID |
| `kD` | 0.0 | PID |
| `kFF` | `12.0 / kMaxRpm` | Volts per RPM |
| `kSmartCurrentLimit` | 40/50 | Profile-based |
| `kOpenLoopRampRate` | 0.1 | Seconds |
| `kClosedLoopRampRate` | 0.0 | Seconds |
| `kDebounceTime` | 0.02 | Seconds |
| `kFeedVoltage` | 6.0 | Volts |
| `kPresetDistanceNearMeters` | 3.0 | Preset |
| `kPresetDistanceMidMeters` | 6.0 | Preset |
| `kPresetDistanceFarMeters` | 9.0 | Preset |
| `kPresetDistanceVeryNearMeters` | 2.0 | Preset |
| `kPresetDistanceVeryFarMeters` | 12.0 | Preset |

Distance/Voltage tables:

| Constant | Value |
|---|---|
| `kLowDistanceMeters` | 5.0 |
| `kMidDistanceMeters` | 8.0 |
| `kHighDistanceMeters` | 10.0 |
| `kLowVoltage` | 6.0 |
| `kMidVoltage` | 8.0 |
| `kHighVoltage` | 10.0 |

## Autonomous
`RobotContainer.getAutonomousCommand()` returns:
```
new PrintCommand("No autonomous command configured")
```
So there is no autonomous behavior yet.
