package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.commands.clearIntake;
import frc.robot.auto.autos.commands.closeRobot;
import frc.robot.auto.autos.commands.robotIntake;
import frc.robot.auto.pathing.DriveWithSource;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.controllers.TranslationController;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveIntakeFirst;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.commands.SetFlywheelSpeed;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootManual;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.SwerveDrive;

public class Source5Piece extends SequentialCommandGroup {

    public Source5Piece() {
        super(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             * 
             * // TODO Correct it
             */
            new SetDrivePosition(new Pose2d(Constants.Paths.START_RIGHT, Constants.Paths.START_RIGHT_ANGLE)), //starts at these points (might need to tweak)

            new ShootManual(Shooter.getInstance(), -10),
            
            //Drive to Center Middle Piece and Intake it
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            Constants.Paths.START_RIGHT,
                            Constants.Paths.START_RIGHT_ANGLE,
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(2.38,3.5),              
                            new Rotation2d(Math.toRadians(134)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(8.19, 4.054),
                        new Rotation2d(180),
                            0.5
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
                            new Translation2d(6,4.2),
                            new Rotation2d(Math.toRadians(270)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(4.541,4.25),              
                            new Rotation2d(Math.toRadians(180)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(3.78,5.29),
                        new Rotation2d(Math.toRadians(180)),
                            0.5
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

            false,

            SwerveDrive.getInstance()

            ),
            //Intake the Source Side Middle Piece
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(3.70,5.29),
                            new Rotation2d(Math.toRadians(180)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(5.865,4.135),              
                            new Rotation2d(Math.toRadians(270)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(8.189, 2.533),
                        new Rotation2d(Math.toRadians(270)),
                            0.5
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
                            new Translation2d(6,4.2),
                            new Rotation2d(Math.toRadians(270)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(4.541,4.25),              
                            new Rotation2d(Math.toRadians(180)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(3.78,5.29),
                        new Rotation2d(Math.toRadians(180)),
                            0.5
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

            false,

            SwerveDrive.getInstance()

            ),
            //Intake Source Middle Piece
             new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(3.70,5.29),
                            new Rotation2d(Math.toRadians(180)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(5.865,4.13),              
                            new Rotation2d(Math.toRadians(270)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(8.189,1.811),              
                            new Rotation2d(Math.toRadians(270)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(8.189,0.811),              
                            new Rotation2d(Math.toRadians(270)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(8.189,0.811),
                        new Rotation2d(Math.toRadians(270)),
                            0.5
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
                            new Translation2d(6,4.2),
                            new Rotation2d(Math.toRadians(270)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(4.541,4.25),              
                            new Rotation2d(Math.toRadians(180)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(3.78,5.29),
                        new Rotation2d(Math.toRadians(180)),
                            0.5
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

            false,

            SwerveDrive.getInstance()

            ),
            //Move to Near Source piece
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            new Translation2d(3.649,1.784),
                            new Rotation2d(Math.toRadians(270)),
                            0.5
                        ),
                        new PathPoint(
                            new Translation2d(1.946,4.135),              
                            new Rotation2d(Math.toRadians(180)),
                        0.5    
                        ),
                        new PathPoint(
                            new Translation2d(2.676,4.135),
                        new Rotation2d(Math.toRadians(180)),
                            0.5
                        )
                    )
                ),

            new Intake(IntakeSubsystem.getInstance(), RobotContainer.cam2, Rotation2d.fromDegrees(180)),

            true,

            SwerveDrive.getInstance()

                ),
            //Move to Speaker Source side
            new DriveWithSource(
                new FollowPath(
                    new Path(
                        new PathPoint(
                            Constants.Paths.START_RIGHT,
                            Constants.Paths.START_RIGHT_ANGLE,
                            0.5
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

            false,

            SwerveDrive.getInstance()

            )
        );
    }
}
