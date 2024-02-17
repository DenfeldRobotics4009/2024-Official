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
import frc.robot.auto.paths.rightStart.fourPiece.RightMidCenterMidCenterRightMidRightPath;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class RightMidCenterMidCenterRightMidRight extends SequentialCommandGroup {

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
    private static Command headToShoot = new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1
                    )
                )
            );
    private static Command smallPath = new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(4.531,4.794),
                        new Rotation2d(Math.PI),
                        1
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            );
    private static Command moveUnderStage = new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI),
                        1
                    )
                )
            );
    public RightMidCenterMidCenterRightMidRight() {
        // super(

        //     /**
        //      * This command sets the original position of the robot,
        //      * as when the robot powers on it will set its position
        //      * initially to (0, 0)
        //      */
        //     new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT.getX(),Constants.Paths.START_RIGHT.getY(),new Rotation2d(Math.PI/2))), //starts at these points (might need to tweak)

        //     //Shoot Starting Piece while moving back
        //     new FollowPathWithRotationSource(
        //         new Path(            
        //             new PathPoint(
        //                 Constants.Paths.START_RIGHT,               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     //TODO: Find the actual angle for the starting position
        //                 1,   // Speed (m/s)
        //                 clearIntake
        //             ),
        //             new PathPoint(
        //                 new Translation2d(3.649,2.838),               // Starting Position (meters)
        //                 new Rotation2d(0),     // Start Rotation (rad)
        //                 1    // Speed (m/s)
        //             )
        //         ),
        //         new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
        //     ),
        //     // Move under stage
        //     new FollowPath(
        //         new Path(
        //             new PathPoint(
        //                 new Translation2d(3.649,2.838),               // Starting Position (meters)
        //                 new Rotation2d(0),     // Start Rotation (rad)
        //                 1    // Speed (m/s)
        //             ),
        //             new PathPoint(
        //                 new Translation2d(4.865,4.135),               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     // Start Rotation (rad)
        //                 1    // Speed (m/s)
        //             ),
        //             new PathPoint(
        //                 new Translation2d(5.865,4.135),               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 robotIntake
        //             )
        //         )

        //     ),
        //     // Intake MidCenter
        //     new FollowPathWithRotationSource(
        //         new Path(
        //             new PathPoint(
        //                 new Translation2d(8.189,4.635),               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     // Start Rotation (rad)
        //                 1
        //             ),
        //             new PathPoint(
        //                 new Translation2d(8.189,3.635),               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     // Start Rotation (rad)
        //                 1    // Speed (m/s)
        //             )
        //         ),
        //         new Intake(IntakeSubsystem.getInstance())
        //     ),
        //     //Head to shoot
        //     headToShoot,
        //     //Robot essentially stops but has really small path so we can run the command
        //     smallPath,
        //     // Move under stage
        //     moveUnderStage,
        //     // Intake MidCenterRight
        //     new FollowPath(
        //         new Path(            
        //             new PathPoint(
        //                 new Translation2d(5.838, 1.541),               // Starting Position (meters)
        //                 new Rotation2d(Math.PI/2),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)     
        //                 robotIntake
        //             ),
        //             new PathPoint(
        //                 new Translation2d(8.189,2.433),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 robotIntake
        //                 )
        //             )
        //         ),
        //         new Intake(IntakeSubsystem.getInstance())
        //     ),
        //     // Goes from midcenter right to shoot
        //     new FollowPath(
        //         new Path(
        //             new PathPoint(
        //                 new Translation2d(5.838, 1.541),              // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 clearIntake
        //             ),
        //             new PathPoint(
        //                 new Translation2d(3.649,2.838),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 closeRobot
        //             )
        //         ),
        //         new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
        //     ),

        //     // goes from shoot to midright
        //     new FollowPathWithRotationSource(
        //         new Path(            
        //             new PathPoint(
        //                 new Translation2d(5.838, 1.541),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 new PrintCommand("Started in Starting Position")       // Command 
        //             ),
        //             new PathPoint(
        //                 new Translation2d(8.189, 0.73),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 robotIntake       // Command 
        //             )
        //         ),
        //          new Intake(IntakeSubsystem.getInstance())
        //     ),
        //     // goes from midright to shoot
        //     new FollowPathWithRotationSource(
        //         new Path(            
        //             new PathPoint(
        //                 new Translation2d(5.838, 1.541),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1,    // Speed (m/s)
        //                 closeRobot       // Command 
        //             ),
        //             new PathPoint(
        //                 new Translation2d(3.649,2.838),               // Starting Position (meters)
        //                 new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
        //                 1    // Speed (m/s)
        //             )
        //         ),
        //          new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
        //     )
        // );
    }
}
