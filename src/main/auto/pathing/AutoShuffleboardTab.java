// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autos;

/**
 * A singleton object handling autonomous routine
 * shuffleboard data
 */
public class AutoShuffleboardTab {
    static AutoShuffleboardTab instance;

    // Shuffleboard object for selecting autonomous routines
    SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

    // Tab to display autonomous data
    public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    // Entries for auto data
    public static final GenericEntry 
        lookAheadEntry = autoTab.add("Look Ahead", 0)
        .withPosition(0, 4).withSize(20, 4).withWidget("Graph").getEntry(), 
        speedEntry = autoTab.add("State Speed", 0)
        .withPosition(0,0).withSize(10, 4).withWidget("Graph").getEntry(),
        
        /**
         * If this graph is negative, the robot couldn't speed up fast enough
         * If this graph is positive, the robot couldn't slow down fast enough
         * The latter is a problem, the former is not (:
         */
        distanceFromGoalEntry = autoTab.add("Distance to Goal", 0)
        .withPosition(10,0).withSize(10, 4).withWidget("Graph").getEntry(),
        lastCrossedPointEntry = autoTab.add("Last Crossed Point Index", 0)
        .withPosition(20, 2).withSize(5, 1).getEntry();

    /**
     * Constructs auto tab, and initializes autoChooser
     */
    private AutoShuffleboardTab() {

        // Iterate through enum of autos
        for (Autos autoEnum : Autos.values()) {
            // Add all enum objects to autoChooser, with name given by enum type
            // The last added option will remain as default
            autoChooser.addOption(autoEnum.toString(), autoEnum.getSequence());
        }
        
        autoTab.add("Autonomous", autoChooser).withPosition(20, 0).withSize(5, 2);
    }

    /**
     * @return AutoShuffleboardTab instance
     */
    public static AutoShuffleboardTab getInstance() {
        if (instance == null) {
            // Construct if not yet constructed
            instance = new AutoShuffleboardTab();
        }

        return instance;
    }

    /**
     * @return currently selected autonomous routine within shuffleboard
     */
    public SequentialCommandGroup getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
