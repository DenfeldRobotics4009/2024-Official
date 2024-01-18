// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class OdometrySource {

    static ArrayList<OdometrySource> relativeInstances = new ArrayList<OdometrySource>(); 
    static ArrayList<OdometrySource> absoluteInstances = new ArrayList<OdometrySource>(); 

    OdometrySource(OdometryType odometryType) {
        // Add this instance to its associated category
        if (odometryType.equals(OdometryType.kAbsolute)) {absoluteInstances.add(this);}
        else {absoluteInstances.add(this);}
    }

    /**
     * Parses through all instances and corrects
     * relative sources, if data permits. Ideally
     * this should be called periodically, however
     * it is also called when getBestPosition is called.
     */
    public static void update() {
        // Call periodic functions of all instances
        for (OdometrySource instance : relativeInstances) {instance.periodic();}
        for (OdometrySource instance : absoluteInstances) {instance.periodic();}

        // Get average pose2d from absolute instances
        Pose2d assumedBestAbsolutePose2d = processPose2dSet(
            getPose2ds(absoluteInstances).toArray(new Pose2d[0]));
        // Correct relative sources
        for (OdometrySource relativeInstance : relativeInstances) {
            relativeInstance.setPosition(assumedBestAbsolutePose2d);
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

    /**
     * Returns and updates the assumed best corrected position.
     * @return Pose2d calculated from every source
     */
    public static Pose2d getBestPosition() {
        update(); // Update for greatest accuracy

        ArrayList<Pose2d> allPose2ds = getPose2ds(relativeInstances);
        allPose2ds.addAll(getPose2ds(absoluteInstances));

        // Convert to array and calculate
        return processPose2dSet(allPose2ds.toArray(new Pose2d[0]));
            
    }

    /**
     * Sets the position of the relative source
     * @param position
     */
    void setPosition(Pose2d position) {}

    /**
     * Returns the position, with an optional
     * parameter to allow for the non-continuous
     * property of absolute position sources
     */
    abstract Optional<Pose2d> getPosition();

    /**
     * Used to poll sources that need constant updating.
     */
    void periodic() {};
}
