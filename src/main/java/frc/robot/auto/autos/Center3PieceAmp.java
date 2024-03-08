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
import frc.robot.commands.sequences.LowerIntake;
import frc.robot.commands.sequences.TransferSequence;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Center3PieceAmp extends SequentialCommandGroup {
    public Center3PieceAmp() {
        super(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_CENTER,Constants.Paths.START_CENTER_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(
                new LowerIntake(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            0.01,
                            new PathPoint(
                                new Translation2d(1, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(3, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                1
                            ),
                            new PathPoint(
                                new Translation2d(3, 6),
                                new Rotation2d(Math.toRadians(180)),
                                1
                            ),
                            new PathPoint(
                                new Translation2d(2, 5.0),
                                new Rotation2d(Math.toRadians(180)),
                                0.5
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()
                )
            ),

            new ParallelCommandGroup(
                new TransferSequence(),
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(2, 5.0),
                                new Rotation2d(Math.toRadians(180)),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(1, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(0.8701, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                0.2
                            )
                        )
                    ),

                    SwerveDrive.getInstance()
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(
                new LowerIntake(),

                new DriveWithSource(
                    new FollowPath(
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
                                new Translation2d(2.26, 6.95),
                                new Rotation2d(Math.toRadians(180)),
                                0.2
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()
                )
            ),

            new ParallelCommandGroup(
                new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(2.26, 6.95),
                                new Rotation2d(Math.toRadians(180)),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(1, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(0.8701, 5.528),
                                new Rotation2d(Math.toRadians(180)),
                                0.2
                            )
                        )
                    ),

                    SwerveDrive.getInstance()
                )
            ),

            new ShootManual(Shooter.getInstance(), -10)
        );
    }
}
