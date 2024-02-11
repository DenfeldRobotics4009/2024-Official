package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveMotors {
    public CANSparkMax DriveMotor, SteerMotor;
    public CANcoder SteerEncoder;
    
    public static double rotationsToMeters(double rotations) {
        return rotations * SwerveModule.rotationsToMeters;
    }
    
    public static double metersToRotations(double meters) {
        return meters / SwerveModule.rotationsToMeters;
    }

    public SwerveMotors(
        int driveMotorID, 
        int steerMotorID, 
        int CANCoderID
    ) {
        this.DriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.SteerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        this.SteerEncoder = new CANcoder(CANCoderID);
    }

    /**
     * @param A Rotation2d A
     * @param B Rotation2d B
     * @return the angle from A to B
     * on the interval [pi, -pi), in radians
     */
    public static Rotation2d signedAngleBetween(Rotation2d A, Rotation2d B) {
        return new Rotation2d(
            (B.getRadians() - A.getRadians() + Math.PI) % (Math.PI * 2) - Math.PI
        );
    }

    /**
     * @return A Rotation2d of the swerve module direction.
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(
            2 * Math.PI * SteerEncoder.getAbsolutePosition().getValueAsDouble()
        );
    }

    /**
     * @return A double of the total drive distance in meters
     */
    public double getDriveDistanceMeters() {
        return rotationsToMeters(
            DriveMotor.getEncoder().getPosition()
        );
    }

    /**
     * @return Current speed of drive motors
     */
    public double getDriveSpeedMeters() {
        return rotationsToMeters
            (DriveMotor.getEncoder().getVelocity()
        );
    }
}