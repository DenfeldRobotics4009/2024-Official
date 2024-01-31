// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A handler for all odometry inputs
 */
public class OdometryHandler {

    static ArrayList<OdometrySource> internal = new ArrayList<OdometrySource>(); 

    static ArrayList<OdometrySource> external = new ArrayList<OdometrySource>(); 
    
    /**
     * Inserts the source instance into the odometry handler for consideration
     * when calculating the most accurate robot position.
     * @param source OdometrySource instance
     * @param type Internal or external
     */
    public static void addSource(OdometrySource source, OdometryType type) {
        if (type == OdometryType.External) {external.add(source);}
        else if (type == OdometryType.Internal) {internal.add(source);}
        else {
            System.out.println("Could not process odometry source: " + source.toString());
        }
    }

    /**
     * Finds the center Pose2d within the given set.
     * 
     * #TODO There is a better way to do this, one that
     * considers the possibility of outliers and bad data.
     * 
     * @param pose2ds
     * @return center Pose2d
     */
    static Pose2d processPose2dSet(Pose2d ... pose2ds) {

        // Check if there is no relevant data
        if (pose2ds.length == 0) {return null;}

        // There likely would be a better way to do this, one
        // that actually considers the possibility of garbage data
        // outliers. Though this is the best I can come up with on the spot.
        Translation2d translation2dSum = new Translation2d();
        Rotation2d rotation2dSum = new Rotation2d();
        for (Pose2d pose2d : pose2ds) {
            translation2dSum = translation2dSum.plus(pose2d.getTranslation());
            rotation2dSum = rotation2dSum.plus(pose2d.getRotation());
        }

        // Divide by length of passed in values and return
        return new Pose2d(
            translation2dSum.div(pose2ds.length), 
            rotation2dSum.div(pose2ds.length)
        );
    }

    /**
     * @param odometrySources
     * @return ArrayList of Pose2ds from each odometrySource
     */
    static ArrayList<Pose2d> getPose2ds(ArrayList<OdometrySource> odometrySources) {
        ArrayList<Pose2d> pose2ds = new ArrayList<Pose2d>();
        for (OdometrySource odometrySource : odometrySources) {
            // If the pose is present, add it to the set
            Optional<Pose2d> pose = odometrySource.getPosition();
            if (pose.isPresent()) {pose2ds.add(pose.get());}
        }
        
        return pose2ds;
    }

    static void correctInternalSources() {
        ArrayList<Pose2d> externalPos = getPose2ds(external);
        if (!externalPos.isEmpty()) {
            Pose2d processed = processPose2dSet(externalPos.toArray(new Pose2d[0]));
            for (OdometrySource internalSource : internal) {
                internalSource.setPosition(processed);
            }
        }
    }

    /**
     * Returns and updates the assumed best corrected position.
     * @return Pose2d calculated from every source
     */
    public static Pose2d getBestPosition() {
        // Correct position of internal sources
        correctInternalSources();

        // Add all the internal sources for processing
        ArrayList<Pose2d> allPositions = new ArrayList<Pose2d>();
        allPositions.addAll(getPose2ds(external));
        allPositions.addAll(getPose2ds(internal));

        // Convert to array and calculate
        return processPose2dSet(allPositions.toArray(new Pose2d[0]));
            
    }
}
