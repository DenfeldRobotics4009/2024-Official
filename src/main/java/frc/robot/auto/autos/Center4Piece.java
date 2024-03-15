// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.pathing.DriveWithSource;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.Field;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.Intake;
import frc.robot.commands.ShootManual;
import frc.robot.commands.sequences.DrivePath;
import frc.robot.commands.sequences.DriveWhileIntaking;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Center4Piece extends SequentialCommandGroup {
    public Center4Piece() {
        super(
            new SetDrivePosition(
                    Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_CENTER,Constants.Paths.START_CENTER_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),

            new DriveWhileIntaking(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(3, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(3, 6),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        0.5
                    )
                ),

                Rotation2d.fromDegrees(180)
            ),

            new DrivePath(
                new Path(
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWhileIntaking(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(1, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(2, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                ),

                Rotation2d.fromDegrees(180)
            ),

            new DrivePath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.26, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWhileIntaking(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(1,5),
                        new Rotation2d(Math.toRadians(150)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(2, 4.2),
                        new Rotation2d(Math.toRadians(150)),
                        0.2
                    )
                ),

                Rotation2d.fromDegrees(180)
            ),

            new DrivePath(
                new Path(
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(0.8701, 5.2), 
                        new Rotation2d(Math.toRadians(180)),
                        0.5
                    )
                )
            ),
            
            new ShootManual(Shooter.getInstance(), -10)
        );
    }
}
