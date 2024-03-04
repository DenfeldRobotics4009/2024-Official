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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  DutyCycleEncoder rotateEncoder = new DutyCycleEncoder(3);

  static IntakeSubsystem instance;

  public double goalIntakePosition = intakePosition.STARTING.get();

  public PIDController intakePIDController = new PIDController(3.75, 0, 0);

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

    rotateMotor.setOpenLoopRampRate(0.4);
    rotateMotor.getEncoder().setPosition(0);

    intakeMotor.setOpenLoopRampRate(0.5);
    // rotateEncoder.reset();

    rotateEncoder.setPositionOffset(Constants.Intake.rotateEncoderOffset);
    rotateEncoder.setDistancePerRotation(1);

    intakePIDController.setTolerance(Constants.Intake.pidTolerance);
  }

  @Override
  public void periodic() {
    
    // Catch limits from switches, calibrate encoder from values
    if (intakeInnerLimitSwitch.get()) {
      if (!intakeInnerSwitchToggle) {
        // The arm is fully pulled into the robot
        rotateEncoder.reset();
        intakeInnerSwitchToggle = true;
      }
    } else {
      intakeInnerSwitchToggle = false;
    }

    if (rotateEncoder.getDistance() >= 0) {
      rotateEncoder.setPositionOffset(rotateEncoder.getPositionOffset() + 1);
    }

    if (rotateEncoder.getDistance() <= -1) {
      rotateEncoder.setPositionOffset(rotateEncoder.getPositionOffset() - 1);
    }


    SmartDashboard.putNumber("intake angle", rotateEncoder.getDistance());
    SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());
    rotateMotor.set(intakePIDController.calculate(rotateEncoder.getDistance()));
  }

  public void setIntake() {
    intakeMotor.set(Constants.Intake.intakeMotorPower);
  }
  public void setIntake(double power) {
    intakeMotor.set(power);
  }
  public void setOuttake(){
    intakeMotor.set(1);
  }
  public void stop() {
    intakeMotor.set(0);
  }

  public boolean atTargetAngle() {
    return intakePIDController.atSetpoint();
  }

  public double getTargetAngle() {
    return intakePIDController.getSetpoint();
  }

  public boolean getIntakeSensor() {
    return intakeLaserSensor.getVoltage() < Constants.laserSensorVoltageHigh;
  }

  public void setPosition(intakePosition position){
    setPosition(position.get());
  }

  public double getPosition() {
    return rotateEncoder.getDistance();
  }

  public boolean atAngle(intakePosition position) {
    return getTargetAngle() == position.get() && atTargetAngle();
  }

  public void setPosition(double position){
    goalIntakePosition = position;
    MathUtil.clamp(position, intakePosition.GROUND.get(), intakePosition.STARTING.get());
    intakePIDController.setSetpoint(position);
  }
}
