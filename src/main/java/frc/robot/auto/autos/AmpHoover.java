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
public class AmpHoover extends SequentialCommandGroup {
    public AmpHoover() {
        super(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_LEFT, Constants.Paths.START_LEFT_ANGLE)
                )
            ),
            
            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                Constants.Paths.START_LEFT,
                                Constants.Paths.START_LEFT_ANGLE,
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(1.135, 7.054),
                                Constants.Paths.START_LEFT_ANGLE,
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(4.054, 6.243),
                                Rotation2d.fromDegrees(180),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(8.19, 7.541), // Far left piece
                                Rotation2d.fromDegrees(180),
                                0
                            )                     
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true, // Race

                    SwerveDrive.getInstance()
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.19, 7.541), // Far left piece
                                Rotation2d.fromDegrees(180),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(8.19, 5.838), // Midleft piece
                                Rotation2d.fromDegrees(135),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(135)),

                    true,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 5.838), // Midleft piece
                            Rotation2d.fromDegrees(135),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 4.054), // Mid note
                            Rotation2d.fromDegrees(135),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(135)),

                true,

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -10),

            new ParallelCommandGroup(

            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.19, 4.054), // Mid note
                                Rotation2d.fromDegrees(135),
                                1.5
                            ),
                            new PathPoint(
                                new Translation2d(8.19, 2.433), // Midright note
                                Rotation2d.fromDegrees(135),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(135)),

                    true,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 2.433), // Midright note
                            Rotation2d.fromDegrees(135),
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 0.811), // Far right note
                            Rotation2d.fromDegrees(135),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(135)),

                true,

                SwerveDrive.getInstance()

            )
        );
    }
}
