package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

 public CANSparkMax armIntakeMotor = new CANSparkMax(Constants.Arm.armIntakeMotorID, MotorType.kBrushless);
 public CANSparkFlex armShooterMotorLeft = new CANSparkFlex(Constants.Arm.armShooterMotorLeftID, MotorType.kBrushless);
 public CANSparkFlex armShooterMotorRight = new CANSparkFlex(Constants.Arm.armShooterMotorRightID, MotorType.kBrushless); 
 public CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
 static Arm instance;
 private double speed = 0;
 public CANcoder rotateEncoder = new CANcoder(63);
    private double target = 0;
    private double measurement = 0;
    private final double kp = 0.25; 
   /**
     * @return singleton instance of Intake
     */
    public static Arm getInstance() {
      if (instance == null) {
        instance = new Arm();
      }
  
      return instance;
    }  
    /** Creates a new Turret. */
  private Arm() {}
  public void setTarget(double target) {
    this.target = MathUtil.clamp(target, Constants.Arm.lowerLimit, Constants.Arm.upperLimit);
  }
  public double getTarget(){
    return target;
  }
  @Override
  public void periodic() {
    measurement = rotateEncoder.getPosition().getValueAsDouble();
    speed = (target - measurement)*kp;
    // This method will be called once per scheduler run
    // Unfinished, make new command with code from the intake command.
    
  }
  /**
   * Sets the speed of the flywheels for the shooter
   * @param percentPower [-1, 1]
   */
  public void setFlyWheelSpeed(double percentPower) {
    armShooterMotorLeft.set(-percentPower);
    armShooterMotorRight.set(percentPower);
  }
   /**
   * Sets the speed of the flywheels for the intake
   * @param percentPower [-1, 1]
   */
  public void setIntake() {
    armIntakeMotor.set(Constants.Arm.armIntakeMotorPower);
  }
  public void setOutake(){
    armIntakeMotor.set(-Constants.Arm.armIntakeMotorPower);
  }
    public void setArmPosition(double percentPower) {
    armShooterMotorLeft.set(-percentPower);
    armShooterMotorRight.set(percentPower);
  }
}