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
import frc.robot.auto.paths.leftStart.threePiece.LeftNearLeftMidLeftPath;
import frc.robot.auto.util.SetDrivePosition;

public class LeftNearLeftMidLeft extends SequentialCommandGroup {
    public LeftNearLeftMidLeft(){
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
                        Constants.Paths.START_CENTER,               // Starting Position
                        new Rotation2d(Math.toRadians(239)),     // Rotation (rad)
                        4,    // Speed (m/s)
                        new PrintCommand("Started here")       // Command 
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
                )
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
