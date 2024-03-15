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

public class Amp2Piece extends SequentialCommandGroup {
    public Amp2Piece() {
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
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(1, 6.95),
                                Constants.Paths.START_LEFT_ANGLE,
                                0.5
                            ),
                            new PathPoint(
                                new Translation2d(2.26, 6.95),
                                new Rotation2d(Math.toRadians(180)),
                                0.2
                            )
                        )
                    ),

                    new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                    true, // Race

                    SwerveDrive.getInstance()
                )
            ),
            new ParallelCommandGroup(

                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(2.26, 6.95),
                                new Rotation2d(Math.toRadians(180)),
                                2
                            ),
                            new PathPoint(
                                Constants.Paths.START_LEFT,
                                Constants.Paths.START_LEFT_ANGLE,
                                0.1
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
