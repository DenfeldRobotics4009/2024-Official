package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.paths.leftStart.twoPiece.LeftNearLeft;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleAuto extends SequentialCommandGroup{
    public ExampleAuto(){
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,7.054,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new LeftNearLeft())
        );
    }
}
