// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.pathing;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.pathing.pathObjects.PathState;

public class FollowPath extends Command {
    
    // Set of processed points
    final Path path;

    // Current index along the path
    public int lastCrossedPointIndex = 0;

    // Distance down the path to drive towards
    public double lookAheadMeters = 0.4;// Initial at 10 cm

    PIDController rotationController = new PIDController(PathingConstants.turningProportion, 0, 0);

    /**
     * Follows a given path
     * @param Path 
     */
    public FollowPath(Path Path) {
        // Assume drive control when path following
        addRequirements(PathingConstants.driveSubsystem);
        path = Path;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Called when the command is initially scheduled.
    @Override
    /**
     * Print path data
     */
    public void initialize() {
        lastCrossedPointIndex = 0;
        lookAheadMeters = 0.4;
        System.out.println("--- Following path of points: ---");
        for (PathPoint point : path.points) {System.out.println(point.posMeters + "," + point.orientation);}
        System.out.println("--- --- --- -- --- -- --- --- ---");

        System.out.println("Scheduling path command: " + getFirstPoint().triggeredCommand);
        getFirstPoint().triggeredCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    /**
     * Operates robot functions from given driveTrain
     */
    public void execute() {

        rotationController.setP(PathingConstants.turningProportion);

        Pose2d robotPose = PathingConstants.driveSubsystem.getPosition();
        PathState state = getPathState(robotPose);
        // The target relative to the robots current position

        System.out.println("Goal location: " + state.goalPose);
        System.out.println("Current location: " + robotPose);

        Translation2d deltaLocation = state.goalPose.getTranslation().minus(robotPose.getTranslation());
        // Clamp state speed so the end of the path can be consistently reached
        // Clamped between [Const Max, 5 cm/s]
        double clampedSpeed = Clamp(
            state.speedMetersPerSecond, 
            PathingConstants.maxVelocityMeters, 
            0.01
        );

        AutoShuffleboardTab.distanceFromGoalEntry.setDouble(deltaLocation.getNorm() - lookAheadMeters);
        AutoShuffleboardTab.speedEntry.setDouble(clampedSpeed);
        AutoShuffleboardTab.lookAheadEntry.setDouble(lookAheadMeters);

        // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
        Translation2d axisSpeeds = new Translation2d(clampedSpeed, deltaLocation.getAngle());

        // Set lookahead based upon speed of next point
        lookAheadMeters = Clamp(
            PathingConstants.lookAheadScalar * clampedSpeed,
            1, 0.2
        );
        // Construct chassis speeds from state values
        // Convert field oriented to robot oriented
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            // Field oriented chassisSpeeds
            new ChassisSpeeds(
                axisSpeeds.getX(),
                axisSpeeds.getY(),

                -rotationController.calculate(
                    robotPose.getRotation().getRadians(), 
                    state.goalPose.getRotation().getRadians()
                )
            ), 
            // Rotate from current direction
            robotPose.getRotation()
        );
        // Drive
        PathingConstants.driveSubsystem.drive(speeds);

        // Recurse until called to end
    }

    
    // Called once the command ends or is interrupted.
    @Override
    /**
     * Triggers final point command
     */
    public void end(boolean interrupted) {
        System.out.println("End of path reached");
        // Schedule last command in path.

        AutoShuffleboardTab.distanceFromGoalEntry.setDouble(0);
        AutoShuffleboardTab.speedEntry.setDouble(0);
        AutoShuffleboardTab.lookAheadEntry.setDouble(0);

        System.out.println("Scheduling path command: " + getLastPoint().triggeredCommand);
        getLastPoint().triggeredCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    /**
     * @return true when the robot is within the last point tolerance
     * and has passed the second to last point.
     */
    public boolean isFinished() {

        // calculate distance to last point
        double distanceToLastPointMeters = getLastPoint().posMeters.getDistance(
            PathingConstants.driveSubsystem.getPosition().getTranslation()
        );

        return (
            // If we have passed the second to last point
            lastCrossedPointIndex >= (path.points.size() - 2) && 
            distanceToLastPointMeters < path.lastPointTolerance
        );
    }

    /**
     * @param robotPosition current pose of the robot
     * @return PathState containing a goal position, rotation
     * and allowed speed to travel there.
     */
    public PathState getPathState(Pose2d robotPosition) {
        //println("Observing line " + lastCrossedPointIndex + " to " + (lastCrossedPointIndex + 1));
        Translation2d robotTranslation = robotPosition.getTranslation();

        // Store 2 points
        ArrayList<PathPoint> relevantPoints = packageRelevantPoints();

        // Assume at least 2 are grabbed
        double lengthAB = relevantPoints.get(0).getDistance(relevantPoints.get(1));
        // println("Length of current line = " + lengthAB);

        // grab perpendicular intersection
        Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
            relevantPoints.get(0).posMeters, relevantPoints.get(1).posMeters, robotTranslation
        );

        // Calculate position along line AB via finding difference between line length, and distance to B
        double distanceMetersAlongAB = lengthAB - relevantPoints.get(1).posMeters.getDistance(perpendicularIntersectionAB);

        // Clamp distance along AB
        if (distanceMetersAlongAB < 0) {
            distanceMetersAlongAB = 0;
        }

        // System.out.println("Distance along AB: " + distanceMetersAlongAB);

        // Calculate look ahead distance from ab, if its over the length, look to BC
        double lookAheadDistanceMetersAlongPoints = distanceMetersAlongAB + lookAheadMeters;

        Translation2d gotoGoal;

        int pointsLookingAhead = 0;
        double distanceAlongLookaheadPoints = lookAheadDistanceMetersAlongPoints;

        // Parse lookahead
        while (true) {
            // Check to make sure points are accessible
            if (lastCrossedPointIndex + pointsLookingAhead + 1 >= path.points.size()) {
                // System.out.println("Looking towards end of path at point ");
                // We are looking to the end of path
                gotoGoal = path.points.get(path.points.size()-1).posMeters;
                //println(gotoGoal);
                break;
            }
            
            // Grab 2 points, and grab the length between them
            PathPoint lookAheadPointA = path.points.get(lastCrossedPointIndex + pointsLookingAhead);
            PathPoint lookAheadPointB = path.points.get(lastCrossedPointIndex + pointsLookingAhead + 1);

            double lookAheadLineLength = lookAheadPointA.getDistance(lookAheadPointB);

            // If we are not looking past this line
            if (distanceAlongLookaheadPoints < lookAheadLineLength) {
                // Stop looping, interpolate goto
                gotoGoal = lookAheadPointA.posMeters.interpolate(
                    lookAheadPointB.posMeters, 
                    distanceAlongLookaheadPoints / lookAheadLineLength // Normalized
                );

                //println(gotoGoal);
                break;
            }

            distanceAlongLookaheadPoints -= lookAheadLineLength;
            // Look 1 line ahead, and subtract length of last line
            pointsLookingAhead ++;
            // Continue
        }

        //println("Constructing path state");
        double percentAlongAB = distanceMetersAlongAB / lengthAB;

        Rotation2d interpolatedRotation = relevantPoints.get(0).orientation.interpolate(
            relevantPoints.get(1).orientation, percentAlongAB);

        // Construct state
        PathState state = new PathState(
            gotoGoal, 

            interpolatedRotation, 
            
            PathPoint.getAtLinearInterpolation(
                relevantPoints.get(0).speedMetersPerSecond, 
                relevantPoints.get(1).speedMetersPerSecond, 
                percentAlongAB
            )
        );

        // Check to increment index
        if (compareWithNextLine(perpendicularIntersectionAB, robotTranslation)) {
            // Schedule associated command
            System.out.println("Scheduling path command: " + relevantPoints.get(1).triggeredCommand);
            relevantPoints.get(1).triggeredCommand.schedule();
            lastCrossedPointIndex ++;
            //println("Increment last crossed point index to " + lastCrossedPointIndex);
        }

        return state;
    }

    public PathPoint getLastPoint() {
        return path.points.get(path.points.size() - 1);
    }

    public PathPoint getFirstPoint() {
        return path.points.get(0);
    }

    /**
     * Returns true if the robot is closer to the next line
     * @param positionAlongLine
     * @param robotTranslation
     * @return false if robot is closer to current line, or next line doesn't exist
     */
    boolean compareWithNextLine(Translation2d positionAlongLine, Translation2d robotTranslation) {
        // Calculate the perpendicularIntersection of the next line in path, if it exists
        // Check if the line exists first
        if (lastCrossedPointIndex + 2 < path.points.size()) {
            //println("Next line found");
            // if the line exists, grab points
            PathPoint pointB = path.points.get(lastCrossedPointIndex + 1);
            PathPoint pointC = path.points.get(lastCrossedPointIndex + 2);
            // grab perpendicular intersection
            Translation2d perpendicularIntersectionBC = PathPoint.findPerpendicularIntersection(
                pointB.posMeters, pointC.posMeters, robotTranslation
            );

            //println("Found perpendicular intersection at " + perpendicularIntersectionBC);
            
            // Find distance from lines
            double distanceToAB = positionAlongLine.getDistance(robotTranslation);
            double distanceToBC = perpendicularIntersectionBC.getDistance(robotTranslation);

            // Compare distances to each intersection
            if (distanceToAB > distanceToBC) {
                //println("Distance to line AB is greater than distance to BC");
                return true;
            }
        } else {
            //println("Next line not found");
        }

        return false;
    }

    /**
     * Construct and return group of 2 relevant points
     * @return Array of PathPoints with length 2
     */
    ArrayList<PathPoint> packageRelevantPoints() {
        // if our lastCrossPoint index is below zero,
        // our path doesn't contain enough points to fit the definition of a path.
        if (lastCrossedPointIndex < 0) {throw new IndexOutOfBoundsException();}

        ArrayList<PathPoint> packagedPoints = new ArrayList<PathPoint>();
        // Try to grab 2 points
        for (int i = 0; i < 2; i++) {
            try {
                packagedPoints.add(
                    path.points.get(lastCrossedPointIndex + i)
                );
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("Could not grab point at index " + lastCrossedPointIndex + i, e.getStackTrace());
            }
        }

        // If we couldn't grab 2 points, decrement.
        if (packagedPoints.size() < 2) {
            lastCrossedPointIndex --;
            return packageRelevantPoints();
        }

        return packagedPoints;
    }

    /**
     * 
     * @param input
     * @param max return max if input > max
     * @param min return min is input < min
     * @return clamped input
     */
    public static double Clamp(double input, double max, double min) {
        if (input > max) {return max;}
        else if (input < min) {return min;}
        else {return input;}
    }

    /**
     * 
     * @param A Rotation2d A
     * @param B Rotation2d B
     * @return the angle from A to B
     * on the interval [pi, -pi), in radians
     */
    public static double signedAngleBetween(Rotation2d A, Rotation2d B) {
        return (
            A.minus(B).getRadians()
        );
    }
}