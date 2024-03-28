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

            //Intake Far Amp Side Piece in the Middle
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            Constants.Paths.START_LEFT,
                            Constants.Paths.START_LEFT_ANGLE,
                            1
                        ),
                        new PathPoint(
                            new Translation2d(1.135, 6.5),
                            Constants.Paths.START_LEFT_ANGLE,
                            3
                        ),
                        new PathPoint(
                            new Translation2d(2.514, 6),
                            Rotation2d.fromDegrees(180),
                            5
                        ),
                        new PathPoint(
                            new Translation2d(7.19, 7.2),
                            Rotation2d.fromDegrees(180),
                            3
                        ),
                        new PathPoint(
                            new Translation2d(8.5, 7.2),
                            Rotation2d.fromDegrees(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                false,

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -145),

            new ParallelCommandGroup(
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.5, 7.2),
                                Rotation2d.fromDegrees(160),
                                3
                            ),
                            new PathPoint(
                                new Translation2d(7, 7.2),
                                Rotation2d.fromDegrees(160),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(7, 5.3), 
                                Rotation2d.fromDegrees(160),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(8, 5.3), // Midleft piece
                                Rotation2d.fromDegrees(160),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(160)),

                    false,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -145),

            new DriveWithSource(
                new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8, 5),
                                Rotation2d.fromDegrees(160),
                                3
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 5.3), // Far left piece
                                Rotation2d.fromDegrees(160),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 3.7), // Midleft piece
                                Rotation2d.fromDegrees(160),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(8, 3.7), // Midleft piece
                                Rotation2d.fromDegrees(160),
                                0
                            )
                        )
                    ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(160)),

                false,

                SwerveDrive.getInstance()

            ),

            new ShootManual(Shooter.getInstance(), -145),

            new ParallelCommandGroup(

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(6.8, 3.7),
                                Rotation2d.fromDegrees(145),
                                3
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 3.7), // Far left piece
                                Rotation2d.fromDegrees(145),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 2.5), // Midleft piece
                                Rotation2d.fromDegrees(145),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(8, 2.5), // Midleft piece
                                Rotation2d.fromDegrees(145),
                                0
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(145)),

                    false,

                    SwerveDrive.getInstance()

                )
            ),

            new ShootManual(Shooter.getInstance(), -145),

            new DriveWithSource(
                new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8, 2.5),
                                Rotation2d.fromDegrees(145),
                                3
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 2.5), // Far left piece
                                Rotation2d.fromDegrees(145),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(6.8, 0.4), // Midleft piece
                                Rotation2d.fromDegrees(145),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(8, 0.4), // Midleft piece
                                Rotation2d.fromDegrees(145),
                                0
                            )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(145)),

                false,

                SwerveDrive.getInstance()

            )
        );
    }
}
