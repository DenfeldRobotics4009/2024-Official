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
//TODO: angles and points guessed
public class SourceHoover extends SequentialCommandGroup {
    public SourceHoover() {
        super(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)
                )
            ),
            
            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                Constants.Paths.START_RIGHT,
                                Constants.Paths.START_RIGHT_ANGLE,
                                3
                            ),
                            new PathPoint(
                                new Translation2d(3.406,1.541),
                                Constants.Paths.START_RIGHT_ANGLE,
                                3
                            ),
                            new PathPoint(
                                new Translation2d(8.19, 0.811), // Far right note
                                new Rotation2d(Math.toRadians(180)),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true, // Race

                    SwerveDrive.getInstance()
                )
            ),
           
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 0.811), // Far right note
                            new Rotation2d(Math.toRadians(180)),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(7, 1.24), // between far right and midright, point guessed
                            new Rotation2d(200),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(
                
            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(7, 1.24), // between far right and midright, point guessed
                                new Rotation2d(200),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(8.19, 2.433), // Midright note
                                new Rotation2d(180),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                )
            ),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 2.433), // Midright note
                            new Rotation2d(180),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(7, 3), // between midright and mid, point guessed
                            new Rotation2d(200),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(7, 3), // between midright and mid, point guessed
                            new Rotation2d(200),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 4.054), // Mid note
                            new Rotation2d(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()

            ),
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 4.054), // Mid note
                            new Rotation2d(180),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(7, 5), // between mid and midleft, point guessed
                            new Rotation2d(200),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(7, 5), // between mid and midleft, point guessed
                            new Rotation2d(200),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 8.838), // Midleft piece, point guessed
                            new Rotation2d(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()
            ),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 8.838), // Midleft piece, point guessed
                            new Rotation2d(180),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(7, 7.25), // between midleft and far left, point guessed
                            new Rotation2d(200),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(7, 7.25), // between midleft and far left, point guessed
                            new Rotation2d(200),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 7.541), // Far left piece, point guessed
                            new Rotation2d(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()
            )

        );
    }
}
