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
import frc.robot.auto.pathing.controllers.TranslationController;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.Field;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.ShootManual;
import frc.robot.commands.TransferIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Amp4Piece extends SequentialCommandGroup {
    public Amp4Piece() {
        super(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_LEFT, Constants.Paths.START_LEFT_ANGLE)
                )
            ),
            
            //Shoot first piece
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
                            new Translation2d(7.19, 7.30), // Tuned from 7.35
                            Rotation2d.fromDegrees(180),
                            3
                        ),
                        new PathPoint(
                            new Translation2d(8.2, 7.30),
                            Rotation2d.fromDegrees(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()

            ),
            //Move to Shoot Point
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 7.35),
                            Rotation2d.fromDegrees(180),
                            4
                        ),
                        new PathPoint(
                            new Translation2d(3.5,6),
                            new Rotation2d(Math.toRadians(180)),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),
            //Shoot
            new DriveWithSource(
                new TranslationController() {

                    @Override
                    public Translation2d getTranslationSpeeds() {
                        return new Translation2d();
                    } 
                },

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),

                true,

                SwerveDrive.getInstance()

            ),

            // Go to Mid Amp Side Piece in the Middle
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(3.22,6.5),
                            new Rotation2d(Math.toRadians(180)),
                            5
                        ),
                        new PathPoint(
                            new Translation2d(7,6.5),
                            new Rotation2d(Math.toRadians(180)),
                            5
                        ),
                        new PathPoint(
                            new Translation2d(7, 5.3),
                            Rotation2d.fromDegrees(180),
                            5
                        ),
                        new PathPoint(
                            new Translation2d(9, 5),
                            Rotation2d.fromDegrees(180),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()

            ),
            //Move to Shoot Point
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.19, 5.838),
                            Rotation2d.fromDegrees(180),
                            4
                        ),
                        new PathPoint(
                            new Translation2d(6.5, 6.5),
                            Rotation2d.fromDegrees(180),
                            4
                        ),
                        new PathPoint(
                            new Translation2d(3.4,6.2),
                            new Rotation2d(Math.toRadians(180)),
                            0
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),
            //Shoot
            new DriveWithSource(
                new TranslationController() {

                    @Override
                    public Translation2d getTranslationSpeeds() {
                        // TODO Auto-generated method stub
                        return new Translation2d();
                    } 
                },

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),

                true,

                SwerveDrive.getInstance()

            ),            

            //Move to Near Amp Side Piece
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(4,6.3),
                            new Rotation2d(Math.toRadians(180)),
                            1
                        ),
                        new PathPoint(
                            new Translation2d(2.26, 6.95),
                            Rotation2d.fromDegrees(60),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(360-60)),

                false,

                SwerveDrive.getInstance()

            ),
            //Shoot
            new DriveWithSource(
                new TranslationController() {

                    @Override
                    public Translation2d getTranslationSpeeds() {
                        // TODO Auto-generated method stub
                        return new Translation2d();
                    } 
                },

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),

                true,

                SwerveDrive.getInstance()

            )
            
        );
    };
}
