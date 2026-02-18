// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/*
One-shot command that sets a shooter RPM target.
It does not wait for spin-up; the subsystem handles control.

Current bindings (RobotContainer):
1. onTrue (one-shot setpoint binding used in multiple places).

Binding impact:
1. onTrue means it sets the target once and ends immediately,
2. if bound with whileTrue in the future, it would keep reasserting the same target.

Termination:
1. ends immediately after initialize() (isFinished returns true),
2. interrupts/cancels are technically possible but not meaningful.
*/
public class SetShooterRpm extends Command {
  // This command sets a shooter speed one time and then ends.
  // Command-based: this command "requires" the shooter subsystem while scheduled.
  private final ShooterSubsystem shooter;
  private final double rpm;

  // One-shot command that sets the shooter target RPM and ends immediately.
  // It does not stop the flywheel; stopping is handled by another command or setpoint.
  public SetShooterRpm(ShooterSubsystem shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    // Requirements prevent two commands from controlling the same subsystem at once.
    addRequirements(shooter);
  }

  // initialize() runs once when the command is scheduled.
  // Set the rpm level.
  @Override
  public void initialize() {
    shooter.setTargetRpm(rpm);
  }

  // Returning true makes this a one-shot command (it ends immediately).
  @Override
  public boolean isFinished() {
    return true;
  }

  // Debug hook so we can see when the command finishes or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end SetShooterRpm command " + (interrupted ? "interrupted" : " "));
  }
}
