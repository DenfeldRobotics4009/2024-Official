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
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class LeftNearLeftMidLeft extends SequentialCommandGroup {
    private static Command closeRobot = new MoveShooterFirst(
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
            new SetDrivePosition(new Pose2d(0, 0,Constants.Paths.START_LEFT_ANGLE)), //starts at these points (might need to tweak)

            // This command will run until the end of the path is reached.
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_LEFT,               // Starting Position
                        Constants.Paths.START_LEFT_ANGLE,
                        4    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(0.5,0),               // Starting Position
                        new Rotation2d(Math.toRadians(239)),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Beginning rotation to 270")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(2,-0.5),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Moving in to grab")       // Command 
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.5,0.5),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        2, // Speed (m/s)
                        new PrintCommand("Initiating transfer and shoot")       // Command 
                ),
                new PathPoint(
                        new Translation2d(3,-0.5),               // Starting Position
                        new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Shoot")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(8.5,-0.4),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(8.5,0),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        2,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(8.5,0.1),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(7,0.1),               // Starting Position
                        new Rotation2d(Math.toRadians(270)),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(3,-0.5),               // Starting Position
                        new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
                        0,    // Speed (m/s)
                        new PrintCommand("Ending")       // Command 
                    )
                )
            )
        );
    }
}
