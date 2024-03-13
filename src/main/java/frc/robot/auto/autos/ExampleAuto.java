package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.FollowPathWithRotationSource;
import frc.robot.auto.autos.commands.clearIntake;
import frc.robot.auto.autos.commands.closeRobot;
import frc.robot.auto.autos.commands.robotIntake;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.Transfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter;

public class ExampleAuto extends SequentialCommandGroup{

    static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    static Shooter shooter = Shooter.getInstance();

    public ExampleAuto(){
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)), //starts at these points (might need to tweak)
            new clearIntake(),

            // This command will run until the end of the path is reached.
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_RIGHT,
                        Constants.Paths.START_RIGHT_ANGLE,
                        1

                        // // Lower intake
                        // new MoveIntakeFirst(
                        //     intake, shooter, 
                        //     intakePosition.GROUND.get(), 
                        //     shooterPosition.GROUND.get()
                        // )
                    ),

                    new PathPoint(
                        new Translation2d(1, -1).plus(Constants.Paths.START_RIGHT),
                        Constants.Paths.START_RIGHT_ANGLE,
                        0

                        // Transfer and reset arms
                        // new SequentialCommandGroup(
                        //     new MoveShooterFirst(
                        //         intake, shooter, 
                        //         intakePosition.DEPOSIT.get(), 
                        //         shooterPosition.DEPOSIT.get()
                        //     ),
                        //     new Transfer(intake, shooter),
                        //     new MoveShooterFirst(
                        //         intake, shooter, 
                        //         intakePosition.STARTING.get(), 
                        //         shooterPosition.GROUND.get()
                        //     )
                        // )
                    )
                ),

                new AutoShoot(shooter, RobotContainer.cam1)
            ),
            // Move under stage
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1,
                        new closeRobot()    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(4.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(5.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new robotIntake()
                    )
                )

            )
        );
    }
}
