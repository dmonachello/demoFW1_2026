package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/*
Starts the flywheel, then feeds only after the shooter is up to speed.
This is the full "shoot" sequence in one command.

Current bindings (RobotContainer):
1. none yet.

Binding impact:
1. with whileTrue, it will run only while held and stop on release,
2. with onTrue, it will run until interrupted or canceled.

Termination:
1. ends when interrupted or canceled by another command,
2. ends on button release when bound with whileTrue,
3. otherwise runs until explicitly canceled.

Behavior:
1. set target RPM on initialize,
2. gate the feeder in execute based on isAtSpeed(),
3. stop feeder (and shooter) when the command ends.
*/
public class AutoShoot extends Command {
  private final ShooterSubsystem shooter;
  private final double targetRpm;
  private final double feedVolts;

  public AutoShoot(ShooterSubsystem shooter, double targetRpm, double feedVolts) {
    this.shooter = shooter;
    this.targetRpm = targetRpm;
    this.feedVolts = feedVolts;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setTargetRpm(targetRpm);
  }

  @Override
  public void execute() {
    if (shooter.isAtSpeed()) {
      shooter.runFeedVoltage(feedVolts);
    } else {
      shooter.stopFeed();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFeed();
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
