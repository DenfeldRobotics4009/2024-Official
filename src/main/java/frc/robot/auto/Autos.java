// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.Field;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.commands.Transfer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.*;

public enum Autos {

    /* ----------------- */
    /* Define Autos here */

    /**
     * Drives the robot along the ExamplePath
     */
    LeftNearLeft(
        new ExampleAuto()
    ),
    CenterNearCenterNearRight( // To tune
       new CenterNearCenterNearRight()
    ),
    LeftNearLeftMidLeft( // To tune
       new LeftNearLeftMidLeft()
    ),
    RightMidCenterMidCenterRightMidRight(
        new RightMidCenterMidCenterRightMidRight()
    ),

    StartLeft(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_LEFT, Constants.Paths.START_LEFT_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    ),

    StartCenter(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_CENTER, Constants.Paths.START_CENTER_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    ),

    StartRight(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)),
            new AutoShoot(Shooter.getInstance(), RobotContainer.cam1),
            new FollowPath(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_CENTER,
                        Constants.Paths.START_CENTER_ANGLE,
                        1
                    ),
                    new PathPoint(
                        Constants.Paths.START_CENTER.plus(new Translation2d(1, 0)),
                        Constants.Paths.START_CENTER_ANGLE,
                        0
                    )
                )
            )
        )
    ),
    DriveForward(
        new SequentialCommandGroup(
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(),
                        new Rotation2d(),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(1,0),
                        new Rotation2d(),
                        0
                    )
                )
            )
        )
    ),
    DriveAndReturn(
        new SequentialCommandGroup(
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(),
                        new Rotation2d(),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(1,0),
                        new Rotation2d(),
                        0
                    )
                )
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(1,0),
                        new Rotation2d(),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(0,0),
                        new Rotation2d(),
                        0
                    )
                )
            )
        )
    ),
    DriveAndReturnWithIntake(
        new SequentialCommandGroup(
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(),
                        new Rotation2d(),
                        1,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(0.8,0),
                        new Rotation2d(Math.toRadians(180)),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(2,0),
                        new Rotation2d(Math.toRadians(180)),
                        0,
                        new SequentialCommandGroup(
                            new MoveShooterFirst(
                                IntakeSubsystem.getInstance(), 
                                Shooter.getInstance(), 
                                intakePosition.DEPOSIT.get(), 
                                shooterPosition.DEPOSIT.get()
                            ),
                            new Transfer(IntakeSubsystem.getInstance(), Shooter.getInstance()),
                            new MoveShooterFirst(
                                IntakeSubsystem.getInstance(), 
                                Shooter.getInstance(), 
                                intakePosition.STARTING.get(), 
                                shooterPosition.GROUND.get()
                            )
      
                        )
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2,0),
                        new Rotation2d(Math.toRadians(180)),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(0,0),
                        new Rotation2d(),
                        0
                    )
                )
            )
        )
    ),
    DriveWithSetPosition(
        new SequentialCommandGroup(
            new SetDrivePosition(new Pose2d(new Translation2d(1, 1), new Rotation2d())),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(1, 1),
                        new Rotation2d(),
                        1
                    ),
                    new PathPoint(
                        new Translation2d(2,1),
                        new Rotation2d(),
                        0
                    )
                )
            )
        )
    ),
    ShootTest(
        new SequentialCommandGroup(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(new Translation2d(0.8701, 5.528), new Rotation2d(Math.toRadians(180)))
                )
            ),
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.DEPOSIT.get(), 
                            shooterPosition.GROUND.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0
                    )
                ),

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            ),
            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
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
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        0.5,
                        new SequentialCommandGroup(
                            new MoveShooterFirst(
                                IntakeSubsystem.getInstance(), 
                                Shooter.getInstance(), 
                                intakePosition.DEPOSIT.get(), 
                                shooterPosition.DEPOSIT.get()
                            ),
                            new Transfer(IntakeSubsystem.getInstance(), Shooter.getInstance()),
                            new MoveShooterFirst(
                                IntakeSubsystem.getInstance(), 
                                Shooter.getInstance(), 
                                intakePosition.DEPOSIT.get(), 
                                shooterPosition.GROUND.get()
                            )
                        )
                    ),
                    new PathPoint(
                        new Translation2d(2.2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        0.5
                    )
                )
            ),

            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(2.2, 5),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    ),
                    new PathPoint(
                        new Translation2d(3, 5),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                ),

                new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            )
        )
    );    
    
    /* ----------------- */

    /**
     * All autonomous routines above will be automatically inserted into
     * the autoChooser object labelled "Autonomous" within the tab "Autonomous".
     */
    final SequentialCommandGroup autoSequence;
    /**
     * Format for creating a new autonomous routine
     * @param autoSequence SequentialCommandGroup of any number of commands
     */
    Autos(SequentialCommandGroup autoSequence) {
        this.autoSequence = autoSequence;
    }

    public SequentialCommandGroup getSequence() {return autoSequence;}
}
