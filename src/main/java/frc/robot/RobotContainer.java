// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.RunFeederManual;
import frc.robot.commands.RunFeederWhenAtSpeed;
import frc.robot.commands.RunShooterUntilStopped;
import frc.robot.commands.SetShooterRpm;
import frc.robot.commands.SetShooterRpmByDistance;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.LookupMode;
import frc.robot.subsystems.ShooterSubsystem.SetpointMode;

/*
Wires together subsystems, commands, and operator controls.
It defines button bindings and exposes the selected autonomous command.

Container responsibilities:
1. construct subsystems and input devices,
2. bind controls to commands,
3. supply the autonomous command.

Command style note:
1. use initialize() for one-shot commands that set a target and end,
2. use execute() for continuous commands that must run every cycle.
*/
public class RobotContainer {
  // CommandScheduler 101 (student version):
  // - Runs every 20 ms and decides which commands are active.
  // - Commands declare which subsystems they "require".
  // - Only one command can require a subsystem at a time.
  // - Scheduling a new command with the same requirement interrupts the old one.
  // - `onTrue(...)` schedules once on press; it does NOT stop on release.
  // - A command ends when `isFinished()` returns true or it is canceled/interrupted.
  // Subsystems are the "hardware owners" in command-based code.
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  // Driver controller (always on port 0).
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OI.kDriverPort);

  // Non-driver controls (either operator Xbox or keypad for now; can be both later).
  private final CommandXboxController operator =
      new CommandXboxController(Constants.OI.kOperatorPort);
  private final CommandGenericHID panel = new CommandGenericHID(Constants.OI.kKeypadPort);  

  public RobotContainer() {
    // Configure all button bindings once at startup.
    configureBindings();
  }

  private void configureBindings() {
    if (Constants.OI.kEnableOperatorXbox) {
      configureOperatorBindings();
    }

    // Keypad bindings for the trellis pad (disabled by default).
    // These show how to bind buttons on a generic HID device.
    if (Constants.OI.kEnableKeypadBindings) {
      configureKeypadBindings();
    }

    // Example: run shooter until limit switch is triggered OR button is released.
    // panel.button(4).whileTrue(
    //   new RunShooterUntilLimitSwitch(shooter, Constants.Shooter.kHighRpm));
    


  }

  private void configureOperatorBindings() {
    // Xbox controller shooter controls.
    // onTrue(...) schedules the command once when the button transitions
    // from not-pressed to pressed (false -> true). It does NOT auto-cancel on release.
    //
    // Scheduler note: If a new command is scheduled that requires the same subsystem,
    // the currently running command is interrupted automatically.
    //
    // SetShooterRpm is a one-shot: initialize() runs once, then isFinished() returns true.
    operator.a().onTrue(new SetShooterRpm(shooter, Constants.Shooter.kLowRpm));
    operator.b().onTrue(new SetShooterRpm(shooter, Constants.Shooter.kMidRpm));

    // RunShooterUntilStopped stays scheduled until interrupted, then its end() stops the shooter.
    // It will be interrupted by any other shooter command (like StopShooter).
    operator.y().onTrue(new RunShooterUntilStopped(shooter, Constants.Shooter.kHighRpm));

    // StopShooter is also a one-shot; scheduling it interrupts any running shooter command.
    operator.x().onTrue(new StopShooter(shooter));

    // Left Bumper: manual feed motor while held, stop on release.
    operator.leftBumper().whileTrue(
        new RunFeederManual(shooter, Constants.Shooter.kFeedVoltage));

    // Right Bumper: gated feed (only runs when shooter is at speed).
    operator.rightBumper().whileTrue(
        new RunFeederWhenAtSpeed(shooter, Constants.Shooter.kFeedVoltage));

    // AutoShoot: spin up, then feed once at speed. Ends on button release.
    operator.leftStick().whileTrue(
        new AutoShoot(shooter, Constants.Shooter.kHighRpm, Constants.Shooter.kFeedVoltage));

    // D-pad distance presets (RPM mode).
    operator.povUp().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceFarMeters, SetpointMode.RPM));
    operator.povLeft().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceMidMeters, SetpointMode.RPM));
    operator.povRight().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceNearMeters, SetpointMode.RPM));

    // Start/Back/Right Stick distance presets (Voltage mode).
    operator.start().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceFarMeters, SetpointMode.VOLTAGE));
    operator.back().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceMidMeters, SetpointMode.VOLTAGE));
    operator.rightStick().onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceNearMeters, SetpointMode.VOLTAGE));

    // Interpolated distance presets (RPM mode) using boolean buttons.
    operator.povDown().onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceMidMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    operator.leftTrigger().onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceNearMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    operator.rightTrigger().onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceFarMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
  }

  private void configureKeypadBindings() {
    // =========================
    // SHOOTER TESTING MAPPINGS
    // =========================
    // This keypad layout is for testing. It includes multiple shooting algorithms
    // so we can compare results and choose a single method for competition.
    //
    // Testing layout (grouped by control type / function):
    // - Open loop (Voltage by distance): buttons 1-5 (Very Near -> Very Far)
    // - Closed loop (RPM presets): buttons 6-10 (Very Low -> Very High)
    // - Closed loop (RPM by distance, nearest): buttons 11-15 (Very Near -> Very Far)
    // - Closed loop (RPM by distance, interpolated): buttons 16-20 (Very Near -> Very Far)
    // - Feeders: 21-22, Stop: 23, Available: 24
    //
    // =============================
    // COMPETITION MAPPING (PLANNED)
    // =============================
    // Once testing is complete, we will collapse this to ONE shooting method:
    // - 1-5: Distance shots (chosen primary method; Very Near -> Very Far)
    // - 6-7: Fallback RPM presets (low, high)
    // - 8: Stop shooter
    // - 9: Gated feed (at speed)
    // - 10: Manual feed (override)
    //
    // Intake (all hold-to-run; no stop button needed):
    // - 11: Intake down
    // - 12: Intake up
    // - 13: Intake forward
    // - 14: Intake reverse
    //
    // Climber (one-press to full travel; requires limits/soft-stops in code):
    // - 15: Climber up
    // - 16: Climber down
    //
    // - 17-24: Available / future

    panel.button(1).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceVeryNearMeters, SetpointMode.VOLTAGE));
    panel.button(2).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceNearMeters, SetpointMode.VOLTAGE));
    panel.button(3).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceMidMeters, SetpointMode.VOLTAGE));
    panel.button(4).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceFarMeters, SetpointMode.VOLTAGE));
    panel.button(5).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceVeryFarMeters, SetpointMode.VOLTAGE));


    panel.button(6).onTrue(new SetShooterRpm(shooter, Constants.Shooter.kVeryLowRpm));
    panel.button(7).onTrue(new SetShooterRpm(shooter, Constants.Shooter.kLowRpm));
    panel.button(8).onTrue(new SetShooterRpm(shooter, Constants.Shooter.kMidRpm));
    panel.button(9).onTrue(new SetShooterRpm(shooter, Constants.Shooter.kHighRpm));
    panel.button(10).onTrue(new SetShooterRpm(shooter, Constants.Shooter.kVeryHighRpm));
    

    // Distance presets (nearest lookup, RPM mode).
    panel.button(11).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceVeryNearMeters, SetpointMode.RPM));
    panel.button(12).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceNearMeters, SetpointMode.RPM));
    panel.button(13).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceMidMeters, SetpointMode.RPM));
    panel.button(14).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceFarMeters, SetpointMode.RPM));
    panel.button(15).onTrue(
        new SetShooterRpmByDistance(
            shooter, Constants.Shooter.kPresetDistanceVeryFarMeters, SetpointMode.RPM));


    // Distance presets (interpolated lookup, RPM mode).
    panel.button(16).onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceVeryNearMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    panel.button(17).onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceNearMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    panel.button(18).onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceMidMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    panel.button(19).onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceFarMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));
    panel.button(20).onTrue(
        new SetShooterRpmByDistance(
            shooter,
            Constants.Shooter.kPresetDistanceVeryFarMeters,
            SetpointMode.RPM,
            LookupMode.INTERPOLATED));

    panel.button(21).whileTrue(
        new RunFeederWhenAtSpeed(shooter, Constants.Shooter.kFeedVoltage));
    panel.button(22).whileTrue(
        new RunFeederManual(shooter, Constants.Shooter.kFeedVoltage));

    panel.button(23).onTrue(new StopShooter(shooter));
    // Button 24: AutoShoot (hold to run).
    panel.button(24).whileTrue(
        new AutoShoot(shooter, Constants.Shooter.kHighRpm, Constants.Shooter.kFeedVoltage));

  }

  public Command getAutonomousCommand() {
    // Return the command to run in autonomous.
    // Right now we just print a message.
    return new PrintCommand("No autonomous command configured");
  }
}
