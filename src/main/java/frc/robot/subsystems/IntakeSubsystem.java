package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {

  public CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);

  static IntakeSubsystem instance;

  // When tripped, there is a piece within the intake
  AnalogInput intakeLaserSensor = new AnalogInput(Constants.Intake.intakeLaserSensorID);

  PneumaticHub lightHub = new PneumaticHub(Constants.Intake.lightHubId);
  Solenoid lightStripA = lightHub.makeSolenoid(14);
  Solenoid lightStripB = lightHub.makeSolenoid(15);


  /**
   * @return singleton instance of Intake
   */
  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }

    return instance;
  }
    /** Creates a new Intake. */
  private IntakeSubsystem() {

    intakeMotor.setOpenLoopRampRate(0.5);
    // rotateEncoder.reset();
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());

    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent()); 

    lightStripA.set(getIntakeSensor() || Shooter.getInstance().getBarrelSensor());
    lightStripB.set(getIntakeSensor() || Shooter.getInstance().getBarrelSensor());
  }

  public void setIntake() {
    intakeMotor.set(Constants.Intake.intakeMotorPower);
  }
  public void setIntake(double power) {
    intakeMotor.set(power);
  }
  public void setOuttake(){
    intakeMotor.set(-0.4);
  }
  public void stop() {
    intakeMotor.set(0);
  }

  public boolean getIntakeSensor() {
    return intakeLaserSensor.getVoltage() < Constants.laserSensorVoltageHigh;
  }

}

/*

if willToLive == 0 {
  die;
}

*/