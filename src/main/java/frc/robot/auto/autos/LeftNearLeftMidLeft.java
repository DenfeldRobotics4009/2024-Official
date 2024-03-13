package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.FollowPathWithRotationSource;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class LeftNearLeftMidLeft extends SequentialCommandGroup {
    private static Command closeRobot = new MoveShooter(
                            IntakeSubsystem.getInstance(),
                            Shooter.getInstance(),
                            intakePosition.STARTING.get(),
                            shooterPosition.GROUND.get()
                        );

    private static Command robotIntake = new MoveIntakeFirst(IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        );
    private static Command clearIntake = new MoveIntakeFirst(IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.DEPOSIT.get(), 
                            shooterPosition.GROUND.get()
                        );
    public LeftNearLeftMidLeft(){
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(Constants.Paths.START_LEFT.getX(),Constants.Paths.START_LEFT.getY(),Constants.Paths.START_LEFT_ANGLE)), //starts at these points (might need to tweak)

            //Robot moves on a path a little so it can shoot.
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_LEFT,
                        Constants.Paths.START_LEFT_ANGLE,
                        1
                    ),
                    new PathPoint(
                        new Translation2d(0.1,7.054),
                        new Rotation2d(0),
                        1
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            // This command will run until the end of the path is reached.
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_LEFT,               // Starting Position
                        Constants.Paths.START_LEFT_ANGLE,
                        4    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(0.5,0),               // Starting Position
                        new Rotation2d(),
                        4    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(2,-0.5),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Moving in to grab")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.5,0.5),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        2, // Speed (m/s)
                        new PrintCommand("Initiating transfer and shoot")       // Command 
                ),
                new PathPoint(
                        new Translation2d(3,-0.5),               // Starting Position
                        new Rotation2d(Math.PI),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Shoot")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(8.5,-0.4),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(8.5,0),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(8.5,0.1),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(7,0.1),               // Starting Position
                        new Rotation2d(Math.PI/2),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(3,-0.5),               // Starting Position
                        new Rotation2d(Math.PI),     // Rotation (rad)
                        0,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    )
                )
            )
        );
    }
}
