package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveInverseKinematics {

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

    Pose2d currentPose;

    private static SwerveDriveInverseKinematics Instance;

    public static SwerveDriveInverseKinematics getInstance(AHRS navxGyro) {
        if (Instance == null) {Instance = new SwerveDriveInverseKinematics(navxGyro);}

        return Instance;
    }

    /**
     * @param SwerveModules 4 Swerve modules to calculate position from
     */
    private SwerveDriveInverseKinematics(AHRS navxGyro) {
        this.navxGyro = navxGyro;
    }
 
    /**
     * Updates current readings from swerve modules to calculate
     * position, should be ran from the drive train's periodic
     * function.
     */
    public void Update() {

        Translation2d wheelPosSum = new Translation2d();
        
        for (SwerveModule swerveModule : SwerveModule.instances) {

            swerveModule.updateFieldRelativePosition(navxGyro.getRotation2d());

            wheelPosSum = wheelPosSum.plus(swerveModule.getFieldRelativePosition());
        }

        currentPose =  new Pose2d (
            wheelPosSum.div(
                SwerveModule.instances.size()
            ), 
            navxGyro.getRotation2d()
        );
    }

    public Pose2d getPosition() {
        return currentPose;
    }

    public void setPosition(Pose2d Position) {
        for (SwerveModule swerveModule : SwerveModule.instances) {
            swerveModule.setFieldRelativePositionFromRobotPosition(Position);
        }
    }
}
