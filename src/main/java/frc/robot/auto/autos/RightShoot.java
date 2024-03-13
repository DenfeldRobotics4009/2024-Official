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
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class RightShoot extends SequentialCommandGroup{
    public RightShoot(){
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_RIGHT,               // Starting Position
                        Constants.Paths.START_RIGHT_ANGLE,
                        1    // Speed (m/s)
                    ),
                    new PathPoint(
                        Constants.Paths.START_RIGHT.plus(new Translation2d(2.5,0)),               // Starting Position
                        new Rotation2d(0),
                        1    // Speed (m/s)
                    )
                )
            )
         );
      }
    }