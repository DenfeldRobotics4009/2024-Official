// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.auto.paths.*;

public enum Autos {

    /* ----------------- */
    /* Define Autos here */

    /**
     * Drives the robot along the ExamplePath
     */
    ExampleAuto(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d()),

            // This command will run until the end of the path is reached.
            new FollowPath(new ExamplePath())
        )
    ),
     StartToSecondNote(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.0,6.0,new Rotation2d(0))),
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new StartToSecondNote()) 
        )
    ),
    OnePieceLeft(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,7.054,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new OnePieceLeft()) 
        )
    ),
    OnePieceCenter(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,4.176,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new OnePieceCenter()) 
        )
    ),
    OnePieceRight(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new OnePieceRight()) 
        )
    ),
    LeftNearLeft(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,7.054,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new LeftNearLeft()) 
        )
    ),
    CenterNearRight(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(2.757,1.946,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new CenterNearRight()) 
        )
    ),  
    RightMidCenterRight(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new RightMidCenterRight()) 
        )
    ),
    LeftnearLeftNearCenter(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new LeftNearLeftNearCenter()) 
        )
    ),
    CenterNearRightNearCenter(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new CenterNearRightNearCenter()) 
        )
    ),
    RightMidCenterMidLeft(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new RightMidCenterMidLeft()) 
        )
    ),
    RightMidCenterMidRight(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new RightMidCenterMidRight()) 
        )
    ),
    LeftNearLeftMidLeft(
        new SequentialCommandGroup(

            /**
             * This command sets the original position of the robot,
             * as when the robot powers on it will set its position
             * initially to (0, 0)
             */
            new SetDrivePosition(new Pose2d(1.946,1.622,new Rotation2d(0))), //starts at these points (might need to tweak)
                //angle to AprilTag and shoot
                new ExampleCommand(new ExampleSubsystem()), //Shooter.getInstance()

            // This command will run until the end of the path is reached.
            new FollowPath(new LeftNearLeftMidLeft()) 
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
