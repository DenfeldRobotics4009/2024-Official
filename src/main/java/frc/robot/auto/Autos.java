// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.*;

public enum Autos {

    /* ----------------- */
    /* Define Autos here */

    /**
     * Drives the robot along the ExamplePath
     */
    LeftNearLeft(
        new ExampleAuto()
    ),
    CenterNearCenterNearRight( // To tune
       new CenterNearCenterNearRight()
    ),
    LeftNearLeftMidLeft( // To tune
       new LeftNearLeftMidLeft()
    ),
    RightMidCenterMidCenterRightMidRight(
        new RightMidCenterMidCenterRightMidRight()
    ),

    StartLeft(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_LEFT, Constants.Paths.START_LEFT_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    ),

    StartCenter(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_CENTER, Constants.Paths.START_CENTER_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    ),

    StartRight(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    );
    /* ----------------- */

    /**
     * All autonomous routines above will be automatically inserted into
     * the autoChooser object labelled "Autonomous" within the tab "Autonomous".
     */
    final SequentialCommandGroup autoSequence;
    /**
     * Format for creating a new autonomous routine
     * @param autoSequence SequentialCommandGroup of any number of commands
     */
    Autos(SequentialCommandGroup autoSequence) {
        this.autoSequence = autoSequence;
    }

    public SequentialCommandGroup getSequence() {return autoSequence;}
}
