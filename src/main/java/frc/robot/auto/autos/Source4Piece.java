package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.MoveShooter;
import frc.robot.commands.SetFlywheelSpeed;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootManual;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.SwerveDrive;

public class Source4Piece extends SequentialCommandGroup {

    public Source4Piece() {
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             * 
             * // TODO Correct it
             */
            new SetDrivePosition(
                Field.mirrorPointIfRed(    
                    new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE) //starts at these points (might need to tweak)
                )
            ),
            new ShootManual(Shooter.getInstance(), -10),
            
            //Drive to Center Middle Piece and Intake it
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        0.8,
                        new PathPoint(
                            Constants.Paths.START_RIGHT,
                            Constants.Paths.START_RIGHT_ANGLE,
                            3
                        ),
                        new PathPoint(
                            new Translation2d(2.38,3.1),              
                            new Rotation2d(Math.toRadians(180)),
                        5
                        ),
                        new PathPoint(
                            new Translation2d(3.2,3.1),              
                            new Rotation2d(Math.toRadians(180)),
                        5
                        ),
                        new PathPoint(
                            new Translation2d(6,4.4),              
                            new Rotation2d(Math.toRadians(180)),
                        5
                        ),
                        new PathPoint(
                            new Translation2d(6.5,4.4),              
                            new Rotation2d(Math.toRadians(180)),
                        3
                        )
                    )
                ),

                SwerveDrive.getInstance()

            ),
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        0.8,
                        new PathPoint(
                            new Translation2d(6.5,4.4),              
                            new Rotation2d(Math.toRadians(180)),
                        3
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 4.4),
                            new Rotation2d(180),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(8.7, 4.4),
                            new Rotation2d(180),
                            0
                        )
                    )
                ), 

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

                true,

                SwerveDrive.getInstance()
            ),
            new ParallelCommandGroup(
                new MoveShooter(Shooter.getInstance(), 0),
                //Move to Shoot Point
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(6,4.2),
                                new Rotation2d(Math.toRadians(180)),
                                4
                            ),
                            new PathPoint(
                                new Translation2d(5.2,4.25), // Tuned from 4.5            
                                new Rotation2d(Math.toRadians(180)),
                            4
                            ),
                            new PathPoint(
                                new Translation2d(3.3,5.29),
                            new Rotation2d(Math.toRadians(180)),
                                0
                            )
                        )
                    ),

                    SwerveDrive.getInstance()

                )
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
            new ParallelCommandGroup(
                new MoveShooter(Shooter.getInstance(), 0),
                //Intake the Source Side Middle Piece
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(3.3,5.29),
                                new Rotation2d(Math.toRadians(180)),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(3.8,5.29),
                                new Rotation2d(Math.toRadians(180)),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(4.6,4.5),
                                new Rotation2d(Math.toRadians(180)),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(6.5,4.5),
                                new Rotation2d(Math.toRadians(180)),
                                2
                            ),
                            new PathPoint(
                                new Translation2d(7,2.8),
                                new Rotation2d(Math.toRadians(360-200)),
                            2

                            )
                        )
                    ),
                
                    SwerveDrive.getInstance()

                )
            ),
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        0.8,
                        new PathPoint(
                            new Translation2d(7,2.8),              
                            new Rotation2d(Math.toRadians(360-200)),
                        3
                        ),
                        new PathPoint(
                            new Translation2d(8.189, 2.8),
                            new Rotation2d(Math.toRadians(360-200)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(8.6, 2.8),
                            new Rotation2d(Math.toRadians(360-200)),
                            0
                        )
                    )
                ),

                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(360-200)),

                true,
            
                SwerveDrive.getInstance()

            ),
            new ParallelCommandGroup(
                new MoveShooter(Shooter.getInstance(), 0),
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(8.189, 2.533),
                                new Rotation2d(Math.toRadians(180)),
                                5
                            ),
                            new PathPoint(
                                new Translation2d(6,4.2),
                                new Rotation2d(Math.toRadians(180)),
                                4
                            ),
                            new PathPoint(
                                new Translation2d(4.9,4.25),              
                                new Rotation2d(Math.toRadians(180)),
                            3

                            ),
                            new PathPoint(
                                new Translation2d(3.3,5.29),
                            new Rotation2d(Math.toRadians(180)),
                                0
                            )
                        )
                    ),

                    SwerveDrive.getInstance()

                )
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
            new ParallelCommandGroup(
                new MoveShooter(Shooter.getInstance(), 0),
                //Intake Source Middle Piece
                new DriveWithSource(
                    new FollowPath(
                        new Path(
                            new PathPoint(
                                new Translation2d(3.70,5.29),
                                new Rotation2d(Math.toRadians(180)),
                                4
                            ),
                            new PathPoint(
                                new Translation2d(4.5,4),              
                                new Rotation2d(Math.toRadians(180)),
                            5
                            ),
                            new PathPoint(
                                new Translation2d(7, 3.7),
                            new Rotation2d(Math.toRadians(135)),
                                5
                            ),
                            new PathPoint(
                                new Translation2d(8.2,1.811),              
                                new Rotation2d(Math.toRadians(100)),
                            5
                            )
                        )
                    ),
                
                    SwerveDrive.getInstance()

                )
            ),
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(8.2,1.811),              
                            new Rotation2d(Math.toRadians(90)),
                        4
                        ),
                        new PathPoint(
                            new Translation2d(8.2,0.811),              
                            new Rotation2d(Math.toRadians(90)),
                        3
                        ),
                        new PathPoint(
                            new Translation2d(8.2,0.3),
                        new Rotation2d(Math.toRadians(90)),
                            0
                        )
                    )
                ),
                
                new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(90)),

                false,

                SwerveDrive.getInstance()

            )
        );
    }
}
