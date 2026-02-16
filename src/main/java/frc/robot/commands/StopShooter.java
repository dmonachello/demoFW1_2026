// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command {
  // This command stops the shooter one time and then ends.
  // Command-based: this command "requires" the shooter subsystem while scheduled.
  private final ShooterSubsystem shooter;

  // Constructor runs once when the command object is created.
  public StopShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
    // Requirements prevent two commands from controlling the same subsystem at once.
    addRequirements(shooter);
  }

  // initialize() is called once when the command is scheduled.
  @Override
  public void initialize() {
    shooter.stop();
  }

  // Returning true makes this a one-shot command (it ends immediately).
  @Override
  public boolean isFinished() {
    return true;
  }
}
