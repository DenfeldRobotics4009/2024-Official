package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModule {

    public final SwerveMotors swerveMotors;
    public final Translation2d robotTrackPosition;
    public final String name;

    // The size of a rectangle representing the robot collected from the 
    // modules furthest from a track position of (0,0)
    private static Translation2d robotTrack = new Translation2d(-1, -1); // Provided at init

    // Default values are according to the SDS MK4I
    // - Hardware defined variables --
    public static double driveRampRate = 0.1;
    public static double steerRampRate = 0.1;
    /**
     * Maximum speed the robot can drive, set by user.
     */
    public static double maxMetersPerSecond = 5.05;
    /**
     * Maximum speed the robot can rotate, calculated from the
     * constructed swerve module positions.
     */
    public static double maxRadPerSecond = 1;
    public static double steerProportion = 0.3;
    public static double wheelDiameterMeters = 0.10308;
    public static double driveGearRatio = 6.2; // Rotations of motor per 1 rotation of the wheel
    public static double rotationsToMeters = (Math.PI * wheelDiameterMeters / driveGearRatio); 
    // -

    // Units in meters
    private double lastAccumulatedDriveDistance = 0;

    /**
     * Initially set by odometry source constructor,
     * if not set, begin at zero
     */
    private Translation2d AccumulatedRelativePositionMeters = new Translation2d();

    final GenericEntry calibrationEntry;

    public static ShuffleboardTab swerveModuleTab = Shuffleboard.getTab("Swerve Modules");

    public static ShuffleboardLayout calibrationAngleEntryGroup = 
        swerveModuleTab.getLayout("Calibration Angle (Degrees)", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(5, 4);

    public static ArrayList<SwerveModule> instances = new ArrayList<SwerveModule>();

    /**
     * 
     * @param swerveMotors
     * @param robotTrackPosition
     * @param name
     */
    public SwerveModule(SwerveMotors swerveMotors, Translation2d robotTrackPosition, String name) {
        instances.add(this);
        this.name = name;
        this.swerveMotors = swerveMotors;
        this.robotTrackPosition = robotTrackPosition;

        // Set motor ramp rates
        swerveMotors.DriveMotor.setOpenLoopRampRate(driveRampRate);
        swerveMotors.SteerMotor.setOpenLoopRampRate(steerRampRate);

        // Check if these values have been kept equal, otherwise
        // the swerve track base isn't rectangular, and we should print a warning.
        if ( !abs(robotTrackPosition).times(2).equals(robotTrack) && (robotTrack.getX() != -1 || robotTrack.getY() != -1) ) {
            DriverStation.reportWarning(
                "Swerve modules constructed with a non-rectangular position," + 
                " disregard this warning if this geometry intentional", false);
        }

        // Set robot track size to largest recorded X and Y components
        if (abs(robotTrackPosition).times(2).getX() > robotTrack.getX()) {
            robotTrack = new Translation2d(abs(robotTrackPosition).times(2).getX(), robotTrack.getY());
            System.out.println("Set Robot track size to " + robotTrack);
        }
        if (abs(robotTrackPosition).times(2).getY() > robotTrack.getY()) {
            robotTrack = new Translation2d(robotTrack.getX(), abs(robotTrackPosition).times(2).getY());
            System.out.println("Set Robot track size to " + robotTrack);
        }

        // Set maximum rotation speed
        maxRadPerSecond = maxMetersPerSecond / Math.hypot(robotTrack.getX() / 2.0, robotTrack.getY() / 2.0);

        // Add calibration entry, persistent for safety
        calibrationEntry = calibrationAngleEntryGroup
            .add(name + " Calibration", swerveMotors.defaultAngleOffset.getDegrees()).getEntry();
    }

    /**
     * Configures the maximum speed of the swerve module in meters per second.
     * The default value has been calculated from a Swerve Drive Specialties 
     * MK4I swerve module with a NEO motor attached. For alternate hardware
     * cases, this should be set manually.
     * 
     * This value will effect all swerve modules.
     * 
     * @default 5.05 meters per second
     * 
     * @param maxMetersPerSecond
     */
    public static void setMaxMetersPerSecond(double maxMetersPerSecond) {
        SwerveModule.maxMetersPerSecond = maxMetersPerSecond;
        rotationsToMeters = (Math.PI * wheelDiameterMeters / driveGearRatio);
    }

    /**
     * Configures the size of the swerve module wheel for rotations to meters
     * calculation. The default value is the size of a Swerve Drive Specialties
     * MK4I wheel.
     * 
     * @default 0.10308 meters
     */
    public static void setWheelDiameterMeters(double wheelDiameterMeters) {
        SwerveModule.wheelDiameterMeters = wheelDiameterMeters;
        rotationsToMeters = (Math.PI * wheelDiameterMeters / driveGearRatio);
    }

    /**
     * Configues the number of rotations of the motor for every rotation
     * of the drive wheel. The default value is for a Swerve Drive Specialties
     * MK4I module.
     * 
     * @default 6.2
     * 
     * @param driveGearRatio
     */
    public static void setDriveGearRatio(double driveGearRatio) {
        SwerveModule.driveGearRatio = driveGearRatio;
    }

    /**
     * Configures the proportion of the error (from the goal angle) to
     * set the steer motor speed to. IN general, higher values provide
     * a faster steering motion, while lower provides slower. Faster
     * steering may create overshoot--leading to a wobble while steering.
     * 
     * While the default value may be sufficient, it is worth tuning further
     * for each specific use case.
     * 
     * This value will effect all swerve modules.
     * 
     * @default 0.3 (30%)
     * 
     * @param steerProportion
     */
    public static void setSteerPIDProportion(double steerProportion) {
        SwerveModule.steerProportion = steerProportion;
    }

    /**
     * Configures the rate (in seconds) at which the drive motor will
     * reach maximum speed (100% power).
     * 
     * While the default value may be sufficient, it is worth tuning further
     * for each specific use case.
     * 
     * This will effect all swerve modules.
     * 
     * @default 0.1 seconds
     * 
     * @param driveRampRate
     */
    public static void setDriveRampRateSeconds(double driveRampRate) {
        SwerveModule.driveRampRate = driveRampRate;
        for (SwerveModule instance : instances) {
            instance.swerveMotors.DriveMotor.setOpenLoopRampRate(driveRampRate);
        }
    }

    /**
     * Configures the rate (in seconds) at which the steer motor
     * will reach maximum speed (100% power).
     * 
     * While the default value may be sufficient, it is worth tuning further
     * for each specific use case.
     * 
     * This will effect all swerve modules.
     * 
     * @default 0.1 seconds
     * 
     * @param steerRampRate
     */
    public static void setSteerRampRateSeconds(double steerRampRate) {
        SwerveModule.steerRampRate = steerRampRate;
        for (SwerveModule instance : instances) {
            instance.swerveMotors.SteerMotor.setOpenLoopRampRate(driveRampRate);
        }
    }

    /**
     * Gets array of robot track positions of modules in the
     * order they were initialized.
     */
    public static Translation2d[] getTrackPositions() {
        ArrayList<Translation2d> posList = new ArrayList<Translation2d>();
        for (SwerveModule swerveModule : instances) {
            posList.add(swerveModule.robotTrackPosition);
        }
        return posList.toArray(new Translation2d[posList.size()]);
    }
    
    /**
     * @return The physical location in meters of the swerve module relative
     * to its starting location. Updated by calling updateMovementVector()
     */
    public Translation2d getFieldRelativePosition() {return AccumulatedRelativePositionMeters;}

    /**
     * @return The assumed physical location in meters of the robot relative to
     * its starting location calculated from the field relative position of this
     * individual swerve module.
     */
    public Translation2d getAssumedRobotFieldRelativePosition() {
        return AccumulatedRelativePositionMeters.minus(robotTrackPosition);
    }

    /**
     * Drive the current swerve module using optimization.
     * Inputs are assumed to be on a scale from 0 to 1
     * @param State Un-Optimized state
     */
    public void drive(SwerveModuleState State) {

        // Update calibration from entries
        swerveMotors.defaultAngleOffset = new Rotation2d(Math.toRadians(calibrationEntry.getDouble(0)));

        SwerveModuleState OptimizedState = optimizeState(State, swerveMotors.getRotation2d());

        // Set drive motor
        swerveMotors.DriveMotor.set(
            OptimizedState.speedMetersPerSecond / maxMetersPerSecond
        );

        // Set turn motor
        swerveMotors.SteerMotor.set(
            SwerveMotors.signedAngleBetween(
                OptimizedState.angle, swerveMotors.getRotation2d()
            ).getRadians() * steerProportion
        );
    }

    /**
     * Optimizes the provided state to not turn more than 90 degrees from the current position
     * @param State
     * @param CurrentRotation
     * @return SwerveModuleState
     */
    static SwerveModuleState optimizeState(SwerveModuleState State, Rotation2d CurrentRotation) {
        Rotation2d RotationDifference = State.angle.minus(CurrentRotation);

        SwerveModuleState OptimizedState = State;

        if (Math.abs(RotationDifference.getRadians()) > (Math.PI / 2)) {
            // Reverse wheel direction and reverse wheel speed
            OptimizedState = new SwerveModuleState(
                -State.speedMetersPerSecond, 
                State.angle.plus(new Rotation2d(Math.PI))
            );
        }
    
        return OptimizedState;
    }

    /**
     * Updates and constructs the field position of this swerve module.
     * This should be ran periodically, and as frequently as possible.
     * 
     * @return AccumulatedRelativePositionMeters
     */
    public Translation2d updateFieldRelativePosition(Rotation2d robotOrientation) {
        // Handoff previous value and update
        double lastAccumulatedDriveDistance_h = lastAccumulatedDriveDistance;
        lastAccumulatedDriveDistance = swerveMotors.getDriveDistanceMeters();
        // This velocity is not for an accurate velocity reading, rather to catch a large
        // jump in drive distance. Grabbing velocity from drive motor will not catch this
        // error.
        double velocityFromDriveDistance = lastAccumulatedDriveDistance - lastAccumulatedDriveDistance_h;
        // For an accurate velocity reading, kMotors.DriveMotor.getEncoder().getVelocity()

        // Make sure this error does not exist!
        // Catch velocity error, and reset position with current robot pos
        // Assume a polling rate of 0.2 seconds.
        if (SwerveMotors.metersToRotations(velocityFromDriveDistance * 0.2) > maxMetersPerSecond) {
            // notify on driver station
            System.out.println(
                "Swerve module " + this.toString() + " has encountered a drive motor encoder failure. " +
                "Attempting recalibration from sibling modules"
            );
            // If this is ran, the swerve module needs to be reset
            // Calculate position from all swerve module instances.
            Translation2d posSum = new Translation2d();
            for (SwerveModule swerveModule : instances) {
                // Average from all, as this module hasnt updated yet
                posSum = posSum.plus(
                    swerveModule.getAssumedRobotFieldRelativePosition()
                );
            }

            // Return corrected position.
            return AccumulatedRelativePositionMeters = posSum.div(instances.size() - 1);
            // End function
        }
        
        // Else no velocity error
        // Calculate delta to add to last accumulated position
        Translation2d movementVectorMeters = new Translation2d(
            velocityFromDriveDistance, // Delta of drive distance
            // Sum is bounded by -pi to pi
            swerveMotors.getRotation2d().plus(robotOrientation)
        );
        // Update single module tracking
        // Add last vector and current vector
        return AccumulatedRelativePositionMeters = AccumulatedRelativePositionMeters.plus(movementVectorMeters);
    }

    /**
     * Updates position of swerve module from position of robot
     * 
     * @param Position Field relative position of robot.
     */
    public void setFieldRelativePositionFromRobotPosition(Translation2d Position) {
        AccumulatedRelativePositionMeters = Position.plus(robotTrackPosition);
    }

    
    /**
     * Unsigns the X and Y components of a provided translation
     */
    Translation2d abs(Translation2d translation) {
        return new Translation2d(
            Math.abs(translation.getX()),
            Math.abs(translation.getY())
        );
    }
}
