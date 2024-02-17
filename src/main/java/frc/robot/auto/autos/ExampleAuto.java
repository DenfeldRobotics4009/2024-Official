package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.paths.leftStart.twoPiece.LeftNearLeft;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExampleAuto extends SequentialCommandGroup{
    public ExampleAuto(){
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(0, 0, new Rotation2d(0))), //starts at these points (might need to tweak)

            // This command will run until the end of the path is reached.
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(),
                        1,
                        new MoveIntakeFirst(IntakeSubsystem.getInstance(), null, 0, 0)
                    )
                )
            )
        );
    }
}
