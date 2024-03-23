// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.util.Field;
import frc.robot.auto.util.SetDrivePosition;
import frc.robot.commands.ShootManual;
import frc.robot.commands.sequences.DrivePath;
import frc.robot.commands.sequences.DriveWhileIntaking;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
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

    //2 Piece Autos

    Amp2Piece(new Amp2Piece()),

    Center2Piece(new Center2Piece()),

    Source2Piece(new Source2Piece()),

    //3 Piece Autos
    
    Center3PieceSource(new Center3PieceSource()),

    Center3PieceAmp(new Center3PieceAmp()),

    Amp3Piece(new Amp3Piece()),

    //4 Piece Autos

    Amp4Piece(new Amp4Piece()),

    Center4Piece(new Center4Piece()),

    Source4Piece(new Source4Piece()),

    ExampleAuto(new Example());

    //5 Piece Autos

    //Source5Piece(new Source5Piece()),

    //Source5PieceMiddle(new Source5PieceMiddle()),

    //6 Piece Autos

    //Source6Piece(new Source6Piece()),

    //Hoover Autos

    //AmpHoover(new AmpHoover()), 

    //SourceHoover(new SourceHoover());   
    
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