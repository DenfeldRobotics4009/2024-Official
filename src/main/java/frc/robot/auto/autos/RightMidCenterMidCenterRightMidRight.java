package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class RightMidCenterMidCenterRightMidRight extends SequentialCommandGroup {

    public RightMidCenterMidCenterRightMidRight() {
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             * 
             * // TODO Correct it
             */
            new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)), //starts at these points (might need to tweak)

            //Shoot Starting Piece while moving back
            new FollowPathWithRotationSource(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_RIGHT,               // Starting Position (meters)
                        Constants.Paths.START_RIGHT_ANGLE,
                        1,   // Speed (m/s)
                        new clearIntake()
                    ),
                    new PathPoint(
                        new Translation2d(3.649,1.238),               // Starting Position (meters)
                        Constants.Paths.START_RIGHT_ANGLE,
                        1    // Speed (m/s)
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            // Move under stage
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(3.649,1.238),               // Starting Position (meters)
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

            ),
            // Intake MidCenter
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(8.189,5.135),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1
                    ),
                    new PathPoint(
                        new Translation2d(8.189,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            //Head to shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI),
                        1,
                        new closeRobot()
                    ),
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1,
                        new clearIntake()
                    )
                )
            ),
            //Robot essentially stops but has really small path so we can run the command
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(4.531,4.884),
                        new Rotation2d(Math.PI),
                        1
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            // Move under stage
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1,
                        new closeRobot()
                    ),
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI/2),
                        1
                    )
                )
            ),
            // Intake MidCenterRight
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(8.189, 3.433),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1,    // Speed (m/s)     
                        new robotIntake()
                    ),
                    new PathPoint(
                        new Translation2d(8.189,2.433),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1
                        )
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2),
            // Head to Shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI),
                        1,
                        new closeRobot()
                    ),
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1,
                        new clearIntake()
                    )
                )
            ),
            //Robot essentially stops but has really small path so we can run the command
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(4.531,4.884),
                        new Rotation2d(Math.PI),
                        1
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            //Head to Shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(4.541,4.784),
                        new Rotation2d(Math.PI),
                        1,
                        new closeRobot()
                    ),
                    new PathPoint(
                        new Translation2d(5.865,4.135),
                        new Rotation2d(Math.PI/2),
                        1,
                        new robotIntake()
                    )
                )
            ),
            //Intake MidRight
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(8.189,1.811),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1
                    ),
                    new PathPoint(
                        new Translation2d(8.189,0.811),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            //Zoom to Right Shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(8.189,3.635),
                        new Rotation2d(Math.PI),
                        1,
                        new closeRobot()
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),
                        new Rotation2d(Math.PI),
                        1,
                        new clearIntake()
                    )
                )
            ),
            //Shoot Final Piece
            new FollowPathWithRotationSource(
                new Path(            
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.PI/2),     //TODO: Find the actual angle for the starting position
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_RIGHT,               // Starting Position (meters)    
                        new Rotation2d(Math.PI),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new closeRobot()
                    )
                ),
                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            )

        );
    }
}
