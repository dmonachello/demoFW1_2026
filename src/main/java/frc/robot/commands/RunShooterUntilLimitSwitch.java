package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Runs the shooter at a fixed RPM until the limit switch is pressed.
 */
public class RunShooterUntilLimitSwitch extends Command {
  private final ShooterSubsystem shooter;
  private final double rpm;

  public RunShooterUntilLimitSwitch(ShooterSubsystem shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setTargetRpm(rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return shooter.isLimitSwitchPressed();
  }
}
