package frc.robot.odometry;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveModule;

public class SwerveDriveInverseKinematics implements OdometrySource {

    /**
     * see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     * for further documentation on the SwerveDriveOdometry class.
     * 
     * Zero degrees on the gyroscope represents facing the opposite
     * alliance driver station. As the robot turns left, the gyroscope 
     * value should increase.
     * 
     * see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
     * for further reading on the field coordinate system standard.
     */

    final AHRS navxGyro;

    private static SwerveDriveInverseKinematics Instance;


    public static SwerveDriveInverseKinematics getInstance(AHRS navxGyro) {
        if (Instance == null) {Instance = new SwerveDriveInverseKinematics(navxGyro);}

        return Instance;
    }

    private SwerveDriveInverseKinematics(AHRS navxGyro) {
        OdometryHandler.addSource(this, OdometryType.Internal); 
        
        this.navxGyro = navxGyro;
    }
 
    /**
     * Updates current readings from swerve modules to calculate
     * position, should be ran from the drive train's periodic
     * function.
     */
    public void periodic() {
        for (SwerveModule swerveModule : SwerveModule.instances) {
            swerveModule.updateFieldRelativePosition(navxGyro.getRotation2d());
        }
    }

    /**
     * Sets the navx to the angle
     * @param angle
     */
    void setGyroAngle(Rotation2d angle) {
        navxGyro.setAngleAdjustment(
            navxGyro.getRotation2d().plus(
                new Rotation2d(Math.toRadians(navxGyro.getAngleAdjustment()))
            ).minus(angle).getDegrees()
        );
    }


    @Override
    public Optional<Pose2d> getPosition() {
        Translation2d wheelPosSum = new Translation2d();
        for (SwerveModule swerveModule : SwerveModule.instances) {
            wheelPosSum = wheelPosSum.plus(swerveModule.getFieldRelativePosition());
        }
        Pose2d position = new Pose2d (wheelPosSum.div(SwerveModule.instances.size()), navxGyro.getRotation2d());
        // This will always return a value
        return Optional.ofNullable(position);
    }

    @Override
    public void setPosition(Pose2d position) {
        setGyroAngle(position.getRotation());

        for (SwerveModule swerveModule : SwerveModule.instances) {
            // Rotation2d of 
            swerveModule.setFieldRelativePositionFromRobotPosition(position.getTranslation());
        }
    }
}
