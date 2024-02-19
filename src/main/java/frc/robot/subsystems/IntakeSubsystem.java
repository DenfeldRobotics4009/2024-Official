package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  public double goalIntakePosition = intakePosition.STARTING.get();
  public PIDController intakePIDController = new PIDController(.05, 0.01, 0);

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

    intakePIDController.setTolerance(Constants.Intake.pidTolerance);
  }

  @Override
  public void periodic() {
    // Catch limits from switches, calibrate encoder from values
    if (intakeInnerLimitSwitch.get()) {
      if (!intakeInnerSwitchToggle) {
        // The arm is fully pulled into the robot
        rotateMotor.getEncoder().setPosition(intakePosition.STARTING.get() + 2);
        intakeInnerSwitchToggle = true;
      }
    } else {
      intakeInnerSwitchToggle = false;
    }

    rotateMotor.set(intakePIDController.calculate(rotateMotor.getEncoder().getPosition()));
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

  public double getTargetAngle() {
    return intakePIDController.getSetpoint();
  }

  public boolean getIntakeSensor() {
    System.out.println("Sens v " + intakeLaserSensor.getVoltage());
    return intakeLaserSensor.getVoltage() < Constants.laserSensorVoltageHigh;
  }

  public void setPosition(intakePosition position){
    setPosition(position.get());
  }

  public void setPosition(double position){
    goalIntakePosition = position;
    intakePIDController.setSetpoint(position);
  }
}
