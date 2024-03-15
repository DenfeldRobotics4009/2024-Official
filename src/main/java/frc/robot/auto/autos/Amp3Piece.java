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
import frc.robot.commands.sequences.LowerIntake;
import frc.robot.commands.sequences.TransferSequence;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Amp3Piece extends SequentialCommandGroup {
    public Amp3Piece() {
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
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(1.135, 7.054),
                            Constants.Paths.START_LEFT_ANGLE,
                            1.5
                        ),
                        new PathPoint(
                            new Translation2d(2.514, 6.325),
                            Rotation2d.fromDegrees(180),
                            3
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 7.541),
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
                            new Translation2d(8.19, 7.541),
                            Rotation2d.fromDegrees(180),
                            3
                        ),
                        new PathPoint(
                            new Translation2d(3.22,6.57),
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

                false,

                SwerveDrive.getInstance()

            ),
            //Move to Near Amp Side Piece
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(3.22,6.57),
                            new Rotation2d(Math.toRadians(180)),
                            1
                        ),
                        new PathPoint(
                            new Translation2d(2.26, 6.95),
                            Rotation2d.fromDegrees(-45),
                            1
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(-45)),

                true,

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

                false,

                SwerveDrive.getInstance()

            )
            
        );
    };
}
