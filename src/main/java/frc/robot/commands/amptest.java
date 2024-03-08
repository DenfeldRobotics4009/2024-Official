package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class amptest extends Command {

  Shooter shooter;

  /** Creates a new Shoot. */
  public amptest(
    Shooter shooter
  ) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPosition(-23.66 *2*Math.PI);
    //get flywheels are up to speed
    shooter.ampTest();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}