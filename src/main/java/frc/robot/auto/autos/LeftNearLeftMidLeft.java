package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
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
            new FollowPath(new LeftNearLeftMidLeftPath()) 
        );
    }
}
