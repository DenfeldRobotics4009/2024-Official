package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.FollowPathWithRotationSource;
import frc.robot.auto.autos.commands.robotIntake;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.Field;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.commands.ShootManual;
import frc.robot.commands.Transfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

public class RightNearRightMidCenter extends SequentialCommandGroup {
    public RightNearRightMidCenter() {
        super(
            new SequentialCommandGroup(
                new SetDrivePosition(
                    Field.mirrorPointIfRed(
                        new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)
                    )
                ),
                new ShootManual(Shooter.getInstance(), -10),
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
                            new Translation2d(1.6, 4.8),
                            new Rotation2d(Math.toRadians(160)),
                            0.5
                        )
                    )
                ),
                new FollowPathWithRotationSource(
                    new Path(
                        new PathPoint(
                            new Translation2d(2, 4.2),
                            new Rotation2d(Math.toRadians(160)),
                            0.01
                        ),
                        new PathPoint(
                            new Translation2d(2, 6),
                            new Rotation2d(Math.toRadians(160)),
                            0.01
                        )
                    ),

                    new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
                ),
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(2, 6),
                            new Rotation2d(Math.toRadians(160)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(2, 5),
                            new Rotation2d(Math.toRadians(160)),
                            0.5
                        )
                        
                    )
                ),
                new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(2.38,3.5),               // Starting Position (meters)
                        Rotation2d.fromDegrees(134),
                        4    // Speed (m/s)
                    ),
                    new PathPoint(
                        new Translation2d(4.865,4.2),               // Starting Position (meters)
                        new Rotation2d(Math.PI),     // Start Rotation (rad)
                        2,     // Speed (m/s)
                        new robotIntake()
                    ),
                    new PathPoint(
                        new Translation2d(5,4.2),               // Starting Position (meters)
                        new Rotation2d(Math.PI),     // Start Rotation (rad)
                        1     // Speed (m/s)
                    )
                )
            ),
            // Intake MidCenter
            new FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(5,4.2),               // Starting Position (meters)
                        new Rotation2d(Math.PI),     // Start Rotation (rad)
                        1
                    ),
                    new PathPoint(
                        new Translation2d(7.6,4.2),               // Starting Position (meters)
                        new Rotation2d(Math.PI),     // Start Rotation (rad)
                        0.5    // Speed (m/s)
                    )
                ),
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2)
            ),
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(7.6,4.2),
                        new Rotation2d(Math.PI/2),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(6.5,4.2),
                        new Rotation2d(Math.PI/2),
                        2.5
                    )
                )
            ),
            //Head to shoot
            new FollowPath(
                new Path(
                    new PathPoint(
                        new Translation2d(6.5,4.2),
                        new Rotation2d(Math.PI/2),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(6,4.2),
                        new Rotation2d(Math.PI/2),
                        2.5
                        // new transferPositions()
                    ),
                    new PathPoint(
                        new Translation2d(4.541,4.25),
                        new Rotation2d(Math.PI),
                        2.5
                    ),
                    new PathPoint(
                        new Translation2d(3.78,5.29),
                        new Rotation2d(Math.PI),
                        2.5
                    )
                )
            ),
            //Robot essentially stops but has really small path so we can run the command
            new FollowPath(//FollowPathWithRotationSource(
                new Path(
                    new PathPoint(
                        new Translation2d(3.78,5.29),
                        new Rotation2d(Math.PI),
                        0.1
                    ),
                    new PathPoint(
                        new Translation2d(3.70,5.29),
                        new Rotation2d(Math.PI),
                        0.1
                    )
                )//,
                //new AutoShoot(Shooter.getInstance(), RobotContainer.cam1)
            )
            )
        );
    }
}
