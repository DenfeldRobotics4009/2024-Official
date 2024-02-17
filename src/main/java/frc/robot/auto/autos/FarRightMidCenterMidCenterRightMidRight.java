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

public class FarRightMidCenterMidCenterRightMidRight extends SequentialCommandGroup {

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
    public FarRightMidCenterMidCenterRightMidRight() {
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(0, 0,new Rotation2d(Math.toRadians(239)))), //starts at these points (might need to tweak)

            // Goes from start to shoot
            new FollowPathWithRotationSource(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_FAR_RIGHT,               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,   // Speed (m/s)
                        clearIntake
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(0)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            // goes from shoot to midcenter
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(4.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(0)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(5.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(-Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        robotIntake
                    )
                )

            ),
            // intake midcenter piece
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(8.189,4.635),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(-Math.PI/2)),     // Start Rotation (rad)
                        1
                    ),
                    new PathPoint(
                        new Translation2d(8.189,3.635),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(-Math.PI/2)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )
                ),
                new Intake(IntakeSubsystem.getInstance())

            ),
            // Goes from midcenter to shoot
            new FollowPathWithRotationSource(
                new Path(            
                    new PathPoint(
                        new Translation2d(4.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        closeRobot
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        closeRobot
                    )
                ),

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)

            ),
            // Goes from shoot to midcenterright
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(5.838, 1.541),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)     
                        robotIntake
                    ),
                    new PathPoint(
                        new Translation2d(8.189,2.433),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        robotIntake
                        )
                    )
                ),
                new Intake(IntakeSubsystem.getInstance())
            )
            // Goes from midcenter right to shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(5.838, 1.541),              // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        clearIntake
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        closeRobot
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            ),

            // goes from shoot to midright
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(5.838, 1.541),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Started in Starting Position")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(8.189, 0.73),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        robotIntake       // Command 
                    )
                ),
                 new Intake(IntakeSubsystem.getInstance()),
            ),
            // goes from midright to shoot
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(5.838, 1.541),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        closeRobot       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        closeRobot       // Command 
                    )
                ),
                 new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            )
        );
    }
}
