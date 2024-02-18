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
