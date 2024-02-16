// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing.pathObjects;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A point along a Path
 */
public class PathPoint {

    public Translation2d posMeters;
    public Rotation2d orientation; // Implemented by path constructor

    public double speedMetersPerSecond; // Corrected by path constructor
    //double accelerationDistance; // For deceleration

    public final Command triggeredCommand;

    /**
     * Constructs a new PathPoint with the given statistics. The
     * order of PathPoints constructed will define the order they
     * are driven by the robot.
     * @param PosMeters Position of point on the field in meters
     * @param Orientation The goal orientation of the robot when it reaches this point
     * @param SpeedMetersPerSecond The speed the robot should travel THROUGH this point
     * @param TriggeredCommand A command that should be triggered as the robot passes this point
     */
    public PathPoint(
        Translation2d PosMeters,
        Rotation2d Orientation,
        double SpeedMetersPerSecond,
        Command TriggeredCommand
    ) {
        posMeters = PosMeters;
        triggeredCommand = TriggeredCommand;

        // May be overridden
        speedMetersPerSecond = SpeedMetersPerSecond;
        orientation = Orientation;
    }
    public PathPoint(
        Translation2d PosMeters,
        Rotation2d Orientation,
        double SpeedMetersPerSecond
    ) {
        posMeters = PosMeters;
        triggeredCommand = new InstantCommand();
        // May be overridden
        speedMetersPerSecond = SpeedMetersPerSecond;
        orientation = Orientation;
    }
    /**
     * 
     * @param Initial Initial Value
     * @param Final Final Value
     * @param PercentBetween Position (In Percent) along line starting from 0
     * 
     * @return Calculated value between Initial and Final 
     * that is on the interpolated line function.
     */
    public static double getAtLinearInterpolation(
        double Initial, double Final, double PercentBetween
    ) {
        return (Final - Initial) * PercentBetween + Initial;
    }

    /**
     * 
     * @param Initial Initial Value
     * @param Final Final Value
     * @param Distance Distance between
     * 
     * @return Slope
     */
    public static double getSlopeOfLinearInterpolation(
        double Initial, double Final, double Distance
    ) {
        return (Final - Initial) / Distance;
    }

    /**
     * Returns the coordinates of the intersection between this line,
     * and a perpendicular line that coincides with point Source.
     * @param Source Line to attach perpendicular line to
     * @return Coordinates of intersection between lines
     */
    public static Translation2d findPerpendicularIntersection(Translation2d PointA, Translation2d PointB, Translation2d Source) {

        double deltaX = PointB.getX() - PointA.getX();
        double deltaY = PointB.getY() - PointA.getY();

        Translation2d intersection;

        // Check for edge cases, deltaX being 0 or deltaY being 0

        if (deltaY == 0) {
            //PathFollower.println("Calculating perpendicular intersection from horizontal line");

            intersection = new Translation2d(
                nonSpecifiedClamp(PointA.getX(), PointB.getX(), Source.getX()), PointA.getY()
            );

        } else if (deltaX == 0) {
            //PathFollower.println("Calculating perpendicular intersection from vertical line");

            intersection = new Translation2d(
                PointA.getX(), nonSpecifiedClamp(PointA.getY(), PointB.getY(), Source.getY())
            );

        } else {

            //PathFollower.println("Calculating perpendicular intersection from sloped line");

            double Slope = deltaY / deltaX;

            double xIntercept = (
                    (Source.getX() / (Slope*Slope)) 
                    + (Source.getY() / (Slope)) 
                    - (PointA.getY() / (Slope))
                    + (PointA.getX())
                ) / (1 + 1/(Slope*Slope));

            intersection = new Translation2d(
                xIntercept, 
                Slope*(xIntercept-PointA.getX()) + PointA.getY()
            );
        }
        
        //PathFollower.println("Found perpendicular intersection at " + intersection);
        return intersection;
    }

    public double getDistance(PathPoint Point) {
        return posMeters.getDistance(Point.posMeters);
    }

    /**
     * Clamps an input translation2d into the boundaries specified by the corners
     * @param cornerA
     * @param cornerB
     * @param Input
     * @return Translation2d within boundaries of corner A and B
     */
    public static Translation2d clamp(Translation2d cornerA, Translation2d cornerB, Translation2d Input) {
        return new Translation2d(
            nonSpecifiedClamp(cornerA.getX(), cornerB.getX(), Input.getX()),
            nonSpecifiedClamp(cornerA.getY(), cornerB.getY(), Input.getY())
        );
    }

    /**
     * Constructs a new pathPathPoint via
     * interpolating values from this to
     * finalPoint.
     * @param finalPoint Ending point
     * @param t normalized distance from initial point
     * @param insertedCommand Command to run when point is passed.
     * @return new PathPoint
     */
    public PathPoint interpolate(PathPoint finalPoint, double t, Command insertedCommand) {
        return new PathPoint(
            // Position
            posMeters.interpolate(finalPoint.posMeters, t), 
            // Rotation
            orientation.interpolate(finalPoint.orientation, t),
            // Speed (Unadjusted)
            getAtLinearInterpolation(
                speedMetersPerSecond, finalPoint.speedMetersPerSecond, t),

            insertedCommand
        );
    }

    /**
     * Clamps a given double input within the boundaries of a and b, without
     * caring if a or b is the maximum or minimum.
     * @param a
     * @param b
     * @param input
     * @return input clamped within a and b
     */
    public static double nonSpecifiedClamp(double a, double b, double input) {
        if (a > b) { // a is maximum
            if (input > a) {return a;}
            else if (input < b) {return b;}
        } else { // b is maximum
            if (input > b) {return b;}
            else if (input < a) {return a;}
        }

        return input;
    }
} 
