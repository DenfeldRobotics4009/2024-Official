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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Example extends SequentialCommandGroup {
    public Example() {
        super(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)), 
                        Constants.Paths.START_CENTER_ANGLE
                    )
                )
            ),
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                            Constants.Paths.START_CENTER_ANGLE,
                            1
                        ),
                        new PathPoint(
                            Constants.Paths.START_CENTER.plus(new Translation2d(2, 0)),
                            Constants.Paths.START_CENTER_ANGLE,
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,
                
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

            )
        );
    }
}
