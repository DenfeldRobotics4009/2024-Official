// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootManual;
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
    // LeftNearLeft(
    //     new ExampleAuto()
    // ),
    // CenterNearCenterNearRight( // To tune
    //    new CenterNearCenterNearRight()
    // ),
    // LeftNearLeftMidLeft( // To tune
    //    new LeftNearLeftMidLeft()
    // ),
    // RightNearRightMidCenter(
    //     new RightNearRightMidCenter()
    // ),
    // RightMidCenterMidCenterRightMidRight(
    //     new RightMidCenterMidCenterRightMidRight()
    // ),
    Center3PieceSource(
        new SequentialCommandGroup( 
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_CENTER, Constants.Paths.START_CENTER_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
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
                        1.5,
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
                        new Translation2d(3, 6),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(3, 5.528),
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
            new ShootManual(Shooter.getInstance(), -10),
            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(1,5),
                        new Rotation2d(Math.toRadians(150)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(2, 4.2),
                        new Rotation2d(Math.toRadians(150)),
                        0.2,
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
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
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
                        new Translation2d(0.8701, 5.528), 
                        new Rotation2d(Math.toRadians(180)),
                        0.5
                    )
                )
            ),
            new ShootManual(Shooter.getInstance(), -10)
        )
    ),

    Amp2Piece(
        new SequentialCommandGroup(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_LEFT, Constants.Paths.START_LEFT_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        Constants.Paths.START_LEFT,
                        Constants.Paths.START_LEFT_ANGLE,
                        0.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(1, 6.95),
                        Constants.Paths.START_LEFT_ANGLE,
                        0.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(2.26, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        0.2,
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
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
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
            new ShootManual(Shooter.getInstance(), -10)
        )
    ),
    Source2Piece(
        new SequentialCommandGroup(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), 0),
            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(1, 4.2),
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
                        new Translation2d(2, 4.2),
                        new Rotation2d(Math.toRadians(180)),
                        0.2,
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
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(1.8, 4.5),
                        new Rotation2d(Math.toRadians(160)),
                        0.5
                    ),
                    new PathPoint(
                        Constants.Paths.START_RIGHT,
                        Constants.Paths.START_RIGHT_ANGLE,
                        0.5
                    )
                )
            ),
            new ShootManual(Shooter.getInstance(), -10)
        )
    ),
    Center2Peice(
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
                        1.5,
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
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    )
                )
            ),
            new ShootManual(Shooter.getInstance(), -10)
        )
    ),
    Center3PieceAmp(
        new SequentialCommandGroup(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_CENTER,Constants.Paths.START_CENTER_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
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
                        1.5,
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
            new ShootManual(Shooter.getInstance(), -10),

            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
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
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),

            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.26, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
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
            new ShootManual(Shooter.getInstance(), -10)
        )
    ),

    Center4Piece(
        new SequentialCommandGroup(
            new SetDrivePosition(
                Field.mirrorPointIfRed(
                    new Pose2d(Constants.Paths.START_CENTER,Constants.Paths.START_CENTER_ANGLE)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1,
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
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(3, 6),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
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
                        1,
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
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                )
            ),

            new ShootManual(Shooter.getInstance(), -10),


            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(1, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(3, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),

            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.26, 6.95),
                        new Rotation2d(Math.toRadians(180)),
                        2.5,
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
                        new Translation2d(1, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        0.2
                    )
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
            new FollowPathWithRotationSource(
                new Path(
                    0.01,
                    new PathPoint(
                        new Translation2d(0.8701, 5.528),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
                        new MoveIntakeFirst(
                            IntakeSubsystem.getInstance(), 
                            Shooter.getInstance(), 
                            intakePosition.GROUND.get(), 
                            shooterPosition.DEPOSIT.get()
                        )
                    ),
                    new PathPoint(
                        new Translation2d(1,5),
                        new Rotation2d(Math.toRadians(150)),
                        1.5
                    ),
                    new PathPoint(
                        new Translation2d(2, 4.2),
                        new Rotation2d(Math.toRadians(150)),
                        0.2,
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
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2, 5.0),
                        new Rotation2d(Math.toRadians(180)),
                        1.5,
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
                        new Translation2d(0.8701, 5.2), 
                        new Rotation2d(Math.toRadians(180)),
                        0.5
                    )
                )
            ),
            new ShootManual(Shooter.getInstance(), -10)
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
