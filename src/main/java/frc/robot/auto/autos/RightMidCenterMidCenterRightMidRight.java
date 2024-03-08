package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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
import frc.robot.commands.SetFlywheelSpeed;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootManual;
import frc.robot.commands.sequences.Transfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class RightMidCenterMidCenterRightMidRight extends SequentialCommandGroup {

    public RightMidCenterMidCenterRightMidRight() {
        super(

            // /**
            //  * This command sets the original position of the robot,
            //  * as when the robot powers on it will set its position
            //  * initially to (0, 0)
            //  * 
            //  * // TODO Correct it
            //  */
            // new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)), //starts at these points (might need to tweak)

            // new ShootManual(Shooter.getInstance(), -10),
            // //Shoot Starting Piece while moving back
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             Constants.Paths.START_RIGHT,               // Starting Position (meters)
            //             Constants.Paths.START_RIGHT_ANGLE,
            //             4   // Speed (m/s)
            //         ),
            //         new PathPoint(
            //             new Translation2d(2.38,3.5),               // Starting Position (meters)
            //             Rotation2d.fromDegrees(134),
            //             4    // Speed (m/s)
            //         )
            //     )
            // ),
            // // Move under stage
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(2.38,3.5),               // Starting Position (meters)
            //             Rotation2d.fromDegrees(134),
            //             4    // Speed (m/s)
            //         ),
            //         new PathPoint(
            //             new Translation2d(4.865,4.2),               // Starting Position (meters)
            //             new Rotation2d(Math.PI),     // Start Rotation (rad)
            //             2,     // Speed (m/s)
            //             new robotIntake()
            //         ),
            //         new PathPoint(
            //             new Translation2d(5,4.2),               // Starting Position (meters)
            //             new Rotation2d(Math.PI),     // Start Rotation (rad)
            //             1     // Speed (m/s)
            //         )
            //     )
            // ),
            // // Intake MidCenter
            // new FollowPathWithRotationSource(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(5,4.2),               // Starting Position (meters)
            //             new Rotation2d(Math.PI),     // Start Rotation (rad)
            //             1
            //         ),
            //         new PathPoint(
            //             new Translation2d(7.6,4.2),               // Starting Position (meters)
            //             new Rotation2d(Math.PI),     // Start Rotation (rad)
            //             0.5    // Speed (m/s)
            //         )
            //     ),
            //     new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            // ),
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(7.6,4.2),
            //             new Rotation2d(Math.PI/2),
            //             2.5
            //         ),
            //         new PathPoint(
            //             new Translation2d(6.5,4.2),
            //             new Rotation2d(Math.PI/2),
            //             2.5
            //         )
            //     )
            // ),
            // //Head to shoot
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(6.5,4.2),
            //             new Rotation2d(Math.PI/2),
            //             2.5
            //         ),
            //         new PathPoint(
            //             new Translation2d(6,4.2),
            //             new Rotation2d(Math.PI/2),
            //             2.5
            //             // new transferPositions()
            //         ),
            //         new PathPoint(
            //             new Translation2d(4.541,4.25),
            //             new Rotation2d(Math.PI),
            //             2.5
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             2.5
            //         )
            //     )
            // ),
            // //Robot essentially stops but has really small path so we can run the command
            // new FollowPath(//FollowPathWithRotationSource(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.70,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         )
            //     )//,
            //     //new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            // ),
            // // Move under stage
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(3.70,5.29),
            //             new Rotation2d(Math.PI),
            //             2.5,
            //             new closeRobot()
            //         ),
            //         new PathPoint(
            //             new Translation2d(5.865,4.135),
            //             new Rotation2d(Math.PI/2),
            //             2.5
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189, 2.533),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             0.1//,    // Speed (m/s)     
            //             //new robotIntake()
            //         )
            //     )
            // ),
            // // Intake MidCenterRight
            // new FollowPath(
            //     new Path(            
            //         new PathPoint(
            //             new Translation2d(8.189,2.533),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             0.1
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189,2.433),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             0.1
            //         )
            //     ) // TODO INTAKE
            // ),
            // // Head to Shoot
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(8.189,2.433),
            //             new Rotation2d(Math.PI),
            //             2,
            //             new closeRobot()
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189, 3.433),
            //             new Rotation2d(Math.PI),
            //             4,
            //             new clearIntake()
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         )
            //     )
            // ),
            // //Robot essentially stops but has really small path so we can run the command
            // new FollowPath(//FollowPathWithRotationSource(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.70,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         )
            //     )//,
            //     //new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            // )
            // // Move under stage
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(3.70,5.29),
            //             new Rotation2d(Math.PI),
            //             1,
            //             new closeRobot()
            //         ),
            //         new PathPoint(
            //             new Translation2d(5.865,4.135),
            //             new Rotation2d(Math.PI/2),
            //             1
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189, 3.433),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             1//,    // Speed (m/s)     
            //             //new robotIntake()
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189,1.811),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             1
            //         )
            //     )
            // ),
            // //Intake MidRight
            // new FollowPath(//FollowPathWithRotationSource(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(8.189,0.911),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             1    // Speed (m/s)
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189,0.811),               // Starting Position (meters)
            //             new Rotation2d(Math.PI/2),     // Start Rotation (rad)
            //             1    // Speed (m/s)
            //         )
            //     )//,
            //     //new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            // ),
            // //Zoom to Right Shoot
            // new FollowPath(
            //     new Path(
            //         new PathPoint(
            //             new Translation2d(8.189,0.811),
            //             new Rotation2d(Math.PI),
            //             1,
            //             new closeRobot()
            //         ),
            //         new PathPoint(
            //             new Translation2d(8.189, 3.433),
            //             new Rotation2d(Math.PI),
            //             1,
            //             new clearIntake()
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         )
            //     )
            // ),
            // //Shoot Final Piece
            // new FollowPath(//FollowPathWithRotationSource(
            //     new Path(            
            //         new PathPoint(
            //             new Translation2d(3.78,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         ),
            //         new PathPoint(
            //             new Translation2d(3.68,5.29),
            //             new Rotation2d(Math.PI),
            //             0.1
            //         )
            //     )//,
            //     //new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            // )

        );
    }
}
