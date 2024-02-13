package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {

  public enum intakePosition {
    GROUND (Constants.Intake.ground),
    DEPOSIT (Constants.Intake.deposit),
    STARTING (0);

    private double position;

    private intakePosition(double position) {
      this.position = position;
    }

    public double get() {
      return position;
    }
  }

  public CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
  public CANSparkMax rotateMotor = new CANSparkMax(Constants.Intake.rotateMotorID, MotorType.kBrushless);

  static IntakeSubsystem instance;

  public double currentIntakePosition = intakePosition.STARTING.get();
  public PIDController intakePIDController = new PIDController(.02, 0, 0);

  // When tripped, there is a piece within the intake
  AnalogInput intakeLaserSensor = new AnalogInput(Constants.Intake.intakeLaserSensorID);

  // Limit switch for aim mechanism
  DigitalInput intakeInnerLimitSwitch = new DigitalInput(Constants.Intake.intakeInnerLimitSwitchID);
  boolean intakeInnerSwitchToggle = false;

  DigitalInput intakeOuterLimitSwitch = new DigitalInput(Constants.Intake.intakeOuterLimitSwitchID);
  boolean intakeOuterSwitchToggle = false;
  

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

    rotateMotor.getEncoder().setPosition(0);

    intakePIDController.setTolerance(Constants.Intake.pidTolerance);
  }

  @Override
  public void periodic() {

    double currentIntakePosition = rotateMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Pos", currentIntakePosition);
    intakePIDController.setSetpoint(currentIntakePosition);
    SmartDashboard.putNumber("Goal", currentIntakePosition);

    // For testing
    SmartDashboard.putBoolean("AtIntakeAngle", atTargetAngle());

    rotateMotor.set(intakePIDController.calculate(currentIntakePosition));

    // Catch limits from switches, calibrate encoder from values
    if (intakeInnerLimitSwitch.get()) {
      if (!intakeInnerSwitchToggle) {
        // The arm is fully pulled into the robot
        rotateMotor.getEncoder().setPosition(intakePosition.STARTING.get());
        intakeInnerSwitchToggle = true;
      }
    } else {
      intakeInnerSwitchToggle = false;
    }

    // Catch outer limits
    if (intakeOuterLimitSwitch.get()) {
      if (!intakeOuterSwitchToggle) {
        // The arm is fully extended outside the robot
        rotateMotor.getEncoder().setPosition(intakePosition.GROUND.get());
        intakeOuterSwitchToggle = true;
      }
    } else {
      intakeOuterSwitchToggle = false;
    }
  }

  public void setIntake() {
    intakeMotor.set(Constants.Intake.intakeMotorPower);
  }
  public void setIntake(double power) {
    intakeMotor.set(power);
  }
  public void setOuttake(){
    intakeMotor.set(-Constants.Intake.intakeMotorPower);
  }
  public void stop() {
    intakeMotor.set(0);
  }

  public boolean atTargetAngle() {
    return intakePIDController.atSetpoint();
  }

  public boolean getIntakeSensor() {
    return intakeLaserSensor.getVoltage() > Constants.laserSensorVoltageHigh;
  }

  public void setPosition(intakePosition position){
    currentIntakePosition = position.get();
  }

  public void setPosition(double position){
    currentIntakePosition = position;
  }
}
