// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing.pathObjects;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.pathing.PathingConstants;
import frc.robot.auto.util.Field;
import frc.robot.commands.Drive;

public class Path {

    public final ArrayList<PathPoint> points;

    public final double lastPointTolerance;
    
    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(PathPoint... Points) {
        this(0.02, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points,
     * 0.2 meters is set as the default end point tolerance
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(double lastPointTolerance, PathPoint... Points) {
        this(lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from multiple points
     * @param Points ArrayList<PathPoint> the first point passed
     * into this initializer is the first point along the path.
     * @param lastPointTolerance double in meters
     */
    public Path(double lastPointTolerance, ArrayList<PathPoint> Points) {
        System.out.println();

        System.out.println("Processing path " + this.toString());

        // Flip all points to the corresponding side
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            System.out.println("Flipping point coordinates to red alliance");
            for (PathPoint pathPoint : Points) {
                pathPoint.posMeters = Field.translateRobotPoseToRed(pathPoint.posMeters);
            }
        }

        this.lastPointTolerance = lastPointTolerance;
        points = Points;

        PathPoint firstPoint = points.get(0);

        // Process point data
        // Correct or implement orientation for each point.
        // Handle first point
        if (firstPoint.orientation == null) {
            // Default as 0, if the robot is not at 0 when 
            // the path begins, it will turn.
            firstPoint.orientation = new Rotation2d();
        }

        // Define others rotation
        for (int i = 1; i < points.size(); i++) {
            PathPoint point = points.get(i);
            PathPoint lastPoint = points.get(i-1);
            // If no orientation was initialized
            if (point.orientation == null) {
                // Set to last
                point.orientation = lastPoint.orientation;
            }
        }

        // Increment rotation of each point by the forward direction angle
        for (PathPoint pathPoint : Points) {
            pathPoint.orientation = pathPoint.orientation.plus(PathingConstants.forwardAngle);
        }

        // Parse through a copy, as the original is being edited
        ArrayList<PathPoint> pointsCopy = Points;
        // Parse backward to correct speed of points
        // Parse from back, end at the first
        for (int i = pointsCopy.size()-1; i > 0; i--) {
            
            PathPoint point = pointsCopy.get(i);
            PathPoint previousPoint = pointsCopy.get(i-1);
            double deltaS = point.speedMetersPerSecond - previousPoint.speedMetersPerSecond;
            double deltaD = point.getDistance(previousPoint);
            // in this case, acceleration is negative, deceleration is positive
            double deceleration = -(deltaS / deltaD);
            // Pull max deceleration from constants
            if (deceleration > PathingConstants.maxAccelerationMeters) {

                // Clamp speed
                double previousSpeed = previousPoint.speedMetersPerSecond;
                // This index will remain unaffected
                Points.get(i-1).speedMetersPerSecond = 
                    point.speedMetersPerSecond + deltaD*PathingConstants.maxAccelerationMeters;
                System.out.println(
                    "Clamped speed from " + previousSpeed + " to " + 
                    previousPoint.speedMetersPerSecond
                );

            } else if (deceleration < PathingConstants.maxAccelerationMeters && deltaS < 0) {

                // Insert new point
                // Normalized, deltaS / Swerve.MaxAccelerationMeters is negative
                double percentFromLastPoint =  1 + (deltaS / (PathingConstants.maxAccelerationMeters * deltaD));
                System.out.println(
                    "Inserted new point at index " + i + " at " + percentFromLastPoint*100 + "%");
                // Interpolate between, and set speed to last speed
                PathPoint insertedPoint = previousPoint.interpolate(
                    point, percentFromLastPoint, 
                    new PrintCommand("Passed interpolated speed point at index " + i)
                );
                insertedPoint.speedMetersPerSecond = previousPoint.speedMetersPerSecond;
                points.add(i, insertedPoint);
            }
        }

        // Parse through point and print data
        for (int i = 0; i < points.size(); i++) {
            System.out.print("Point " + i);
            System.out.print(" - " + new Pose2d(points.get(i).posMeters, points.get(i).orientation));
            System.out.println(" - " + points.get(i).speedMetersPerSecond + " meters/sec");
        }

        System.out.println();
    }
} 
