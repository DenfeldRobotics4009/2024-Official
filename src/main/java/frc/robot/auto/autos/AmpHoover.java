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

                new LowerIntake(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                Constants.Paths.START_LEFT,
                                Constants.Paths.START_LEFT_ANGLE,
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(4.054, 6.243),
                                new Rotation2d(180),
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(8.25, 7), // Far left piece, point guessed
                                new Rotation2d(180),
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(8.25, 7), // Far left piece, point guessed
                                new Rotation2d(135),
                                0.2
                            )                        
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true, // Race

                    SwerveDrive.getInstance()
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            new ParallelCommandGroup(
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.25, 6.25), // Midleft piece, point guessed
                                new Rotation2d(135),
                                1
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                    )
                ),

            new ShootManual(Shooter.getInstance(), -10),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            new ParallelCommandGroup(

            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.25, 4.135), // Mid note
                                new Rotation2d(135),
                                1
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                    )
                ),

            new ShootManual(Shooter.getInstance(), -10),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            new ParallelCommandGroup(

            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.25, 4.135), // Mid note
                                new Rotation2d(135),
                                1
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            new ParallelCommandGroup(

            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.25, 2.514), // Midright note
                                new Rotation2d(135),
                                1
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -10),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            new ParallelCommandGroup(

            new TransferSequence(),

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.25, 0.77), // Far right note
                                new Rotation2d(135),
                                1
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true,

                    SwerveDrive.getInstance()

                )
            )
        );
    }
}
