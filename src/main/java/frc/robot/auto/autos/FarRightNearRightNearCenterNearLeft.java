package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.FollowPathWithRotationSource;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Shooter;

public class FarRightNearRightNearCenterNearLeft extends SequentialCommandGroup {
    public FarRightNearRightNearCenterNearLeft(){
        super(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_FAR_RIGHT,new Rotation2d(0))),
            //drive then shoot
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                    Constants.Paths.START_FAR_RIGHT,
                    new Rotation2d(0),
                    5
                    ),
                new PathPoint(
                    new Translation2d(2.108,3.243),
                    new Rotation2d(Math.PI/2),
                    5
                )
                ),
            new AutoShoot(Shooter.getInstance(),RobotContainer.cam1)
            )
        );
    }
}
