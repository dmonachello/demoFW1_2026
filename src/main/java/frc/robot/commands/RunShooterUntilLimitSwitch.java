package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/*
Runs the shooter at a target RPM until the limit switch trips.
Used for automated stop based on a physical sensor.

Current bindings (RobotContainer):
1. example onTrue binding is present but commented out.

Binding impact:
1. when bound with onTrue, it will run until the limit switch ends it,
2. if bound with whileTrue in the future, it would still stop on the switch,
   but also end on button release.
*/
public class RunShooterUntilLimitSwitch extends Command {
  /*
   * Runs the shooter at a fixed RPM until the limit switch is pressed.
   */
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
