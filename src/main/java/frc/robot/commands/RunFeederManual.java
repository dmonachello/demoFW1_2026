package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/*
Manual feeder control while scheduled.
It applies a fixed voltage in execute and stops on end.

Current bindings (RobotContainer):
1. whileTrue (held-run binding used in multiple places).

Binding impact:
1. whileTrue means this command runs only while the button is held,
2. if bound with onTrue in the future, it would run indefinitely until interrupted.

Termination:
1. ends when interrupted or canceled by another command,
2. ends on button release when bound with whileTrue,
3. otherwise runs until explicitly canceled.
*/
public class RunFeederManual extends Command {
  /*
   * Runs the feeder motor at a fixed voltage while scheduled.
   * Intended for manual control (e.g., while a button is held).
   */
  private final ShooterSubsystem shooter;
  private final double volts;

  public RunFeederManual(ShooterSubsystem shooter, double volts) {
    this.shooter = shooter;
    this.volts = volts;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.runFeedVoltage(volts);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
