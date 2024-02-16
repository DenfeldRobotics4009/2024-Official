// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public enum shooterPosition {
    GROUND (0),
    DEPOSIT (Constants.Shooter.transferAngle);

    private double position;

    private shooterPosition(double position) {
      this.position = position;
    }

    public double get() {
      return position;
    }
  }

  public final ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
  GenericEntry rpmEntryTop;
  GenericEntry rpmEntryBottom;

  private CANSparkFlex topMotor = new CANSparkFlex(Constants.Shooter.topMotorID, MotorType.kBrushless);
  private CANSparkFlex bottomMotor = new CANSparkFlex(Constants.Shooter.bottomMotorID, MotorType.kBrushless);
  SparkPIDController topFlywheelPidController = topMotor.getPIDController();
  SparkPIDController bottomFlywheelPidController = bottomMotor.getPIDController();

  private CANSparkMax feeder = new CANSparkMax(Constants.Shooter.feederMotorID, MotorType.kBrushless);

  public CANSparkMax aim = new CANSparkMax(Constants.Shooter.aimMotorID, MotorType.kBrushless);
  private PIDController aimPIDController = new PIDController(.01, 0, 0);
  private double targetAngle = 0;
  static Shooter instance;

  // This sensor detects notes within the barrel
  AnalogInput barrelSensor = new AnalogInput(Constants.Shooter.barrelLaserSensorID);

  // Limit switch for aim mechanism
  DigitalInput aimLimitSwitch = new DigitalInput(Constants.Shooter.aimLimitSwitchID);
  boolean limitSwitchToggle = false;

  /**
   * @return singleton instance of Turret
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  /** Creates a new Turret. */
  private Shooter() {
    aimPIDController.setTolerance(Constants.Shooter.pidTolerance);

    aim.getEncoder().setPosition(0);

    topMotor.setOpenLoopRampRate(0.1);
    bottomMotor.setOpenLoopRampRate(0.1);

    rpmEntryTop = turretTab.add("rpmEntryTop", 5440).withWidget("Number Slider").getEntry();
    rpmEntryBottom = turretTab.add("rpmEntryBottom", 5440).withWidget("Number Slider").getEntry();

    // Set PID values for flywheels using spark maxes, we are
    // using these PID controllers as they support feed forward.
    topFlywheelPidController.setP(Constants.Shooter.flyWheelP);
    topFlywheelPidController.setI(Constants.Shooter.flyWheelI);
    topFlywheelPidController.setD(Constants.Shooter.flyWheelD);
    topFlywheelPidController.setFF(Constants.Shooter.flyWheelF);

    bottomFlywheelPidController.setP(Constants.Shooter.flyWheelP);
    bottomFlywheelPidController.setI(Constants.Shooter.flyWheelI);
    bottomFlywheelPidController.setD(Constants.Shooter.flyWheelD);
    bottomFlywheelPidController.setFF(Constants.Shooter.flyWheelF);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Aim Angle", aim.getEncoder().getPosition()*2*Math.PI);
    double speed = aimPIDController.calculate(aim.getEncoder().getPosition()*2*Math.PI, targetAngle);
    MathUtil.clamp(speed, -1, 1);
    aim.set(speed);

    SmartDashboard.putBoolean("Sensor", getBarrelSensor());

    SmartDashboard.putNumber("Shooter Angle (Radians)", aim.getEncoder().getPosition()*2*Math.PI);

    // Check the limit switch to reset aim encoder
    if (aimLimitSwitch.get()) {
      if (!limitSwitchToggle) {
        aim.getEncoder().setPosition(0);
      }
      limitSwitchToggle = true;
    } else {
      limitSwitchToggle = false;
    }
  }

  public boolean atTargetAngle() {
    return aimPIDController.atSetpoint();
  }

  public double getTargetAngle() {
    return aimPIDController.getSetpoint();
  }

  /**
   * Sets the speed of the flywheels
   * @param percentPower [-1, 1]
   */
  public boolean setFlyWheelSpeed(double rpm) {
    // Set pid values, this automatically drives the motor
    bottomFlywheelPidController.setReference(-rpmEntryTop.getDouble(5440), ControlType.kVelocity);
    topFlywheelPidController.setReference(rpmEntryBottom.getDouble(5440), ControlType.kVelocity);

    // Check tolerance
    return (
      topMotor.getEncoder().getVelocity() >= (rpm-Constants.Shooter.flyWheelTolerance) && 
      bottomMotor.getEncoder().getVelocity() <= (Constants.Shooter.flyWheelTolerance-rpm)
    );
  }
    public void feed() {
    feeder.set(-Constants.Shooter.feederSpeed);
  }
  public void stopFeed() {
    feeder.set(0);
  }

  public void setPosition(double position) {
    targetAngle = MathUtil.clamp(position, Constants.Shooter.aimRangeFrom0, 0);
  }

  public void setPosition(shooterPosition position) {
    targetAngle = position.get();
  }

  public void stopShooter() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public boolean getBarrelSensor() {
    return barrelSensor.getVoltage() < Constants.laserSensorVoltageHigh; 
  }
}
