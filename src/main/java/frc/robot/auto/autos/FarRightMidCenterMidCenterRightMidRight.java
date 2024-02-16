package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.paths.rightStart.fourPiece.RightMidCenterMidCenterRightMidRightPath;
import frc.robot.auto.util.SetDrivePosition;

public class FarRightMidCenterMidCenterRightMidRight extends SequentialCommandGroup {
    public FarRightMidCenterMidCenterRightMidRight() {
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(0, 0,new Rotation2d(Math.toRadians(239)))), //starts at these points (might need to tweak)

            // This command will run until the end of the path is reached.
            new FollowPath(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_FAR_RIGHT,               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(0)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(4.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(0)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(8.189,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(-Math.PI/2)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )

                )
            ),
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(4.865,4.135),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(3.649,2.838),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1    // Speed (m/s)
                    )
                )
            ),
            new FollowPath(
                new Path(            
                    new PathPoint(
                        new Translation2d(),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Started in Starting Position")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(0,0),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Arrived at point 1")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_FAR_RIGHT,               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Started in Starting Position")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(0,0),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Arrived at point 1")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_FAR_RIGHT,               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Started in Starting Position")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(0,0),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Arrived at point 1")       // Command 
                    )
                )
            ),
            new FollowPath(
                new Path(            
                    new PathPoint(
                        Constants.Paths.START_FAR_RIGHT,               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Started in Starting Position")       // Command 
                    ),
                    new PathPoint(
                        new Translation2d(0,0),               // Starting Position (meters)
                        new Rotation2d(Math.toRadians(Math.PI/2)),     // Start Rotation (rad)
                        1,    // Speed (m/s)
                        new PrintCommand("Arrived at point 1")       // Command 
                    )
                )
            )
        );
    }
}
