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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  // Example spark initialization
  // CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);

  private CANSparkFlex topMotor = new CANSparkFlex(Constants.Turret.topMotorID, MotorType.kBrushless);
  private CANSparkFlex bottomMotor = new CANSparkFlex(Constants.Turret.bottomMotorID, MotorType.kBrushless);
  SparkPIDController topFlywheelPidController = topMotor.getPIDController();
  SparkPIDController bottomFlywheelPidController = bottomMotor.getPIDController();

  private CANSparkMax feeder = new CANSparkMax(Constants.Turret.feederMotorID, MotorType.kBrushless);

  private CANSparkMax aim = new CANSparkMax(Constants.Turret.aimMotorID, MotorType.kBrushless);
  private PIDController aimPIDController = new PIDController(.01, 0, 0);
  private double targetAngle = 0;
  static Turret instance;

  /**
   * @return singleton instance of Turret
   */
  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }

    return instance;
  }

  /** Creates a new Turret. */
  private Turret() {
    aimPIDController.setTolerance(Constants.Turret.aimTolerance);

    aim.getEncoder().setPosition(0);

    topMotor.setOpenLoopRampRate(0.1);
    bottomMotor.setOpenLoopRampRate(0.1);

    // Set PID values for flywheels using spark maxes, we are
    // using these PID controllers as they support feed forward.
    topFlywheelPidController.setP(Constants.Turret.flyWheelP);
    topFlywheelPidController.setI(Constants.Turret.flyWheelI);
    topFlywheelPidController.setD(Constants.Turret.flyWheelD);
    topFlywheelPidController.setFF(Constants.Turret.flyWheelF);

    bottomFlywheelPidController.setP(Constants.Turret.flyWheelP);
    bottomFlywheelPidController.setI(Constants.Turret.flyWheelI);
    bottomFlywheelPidController.setD(Constants.Turret.flyWheelD);
    bottomFlywheelPidController.setFF(Constants.Turret.flyWheelF);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Aim Angle", aim.getEncoder().getPosition()*2*Math.PI);
    double speed = aimPIDController.calculate(aim.getEncoder().getPosition()*2*Math.PI, targetAngle);
    MathUtil.clamp(speed, -1, 1);
    aim.set(speed);
    // This method will be called once per scheduler run
  }

  public boolean atTargetAngle() {
    return true; //aimPIDController.atSetpoint();
  }

  /**
   * Sets the speed of the flywheels
   * @param percentPower [-1, 1]
   */
  public boolean setFlyWheelSpeed(double rpm) {
    // Set pid values, this automatically drives the motor
    bottomFlywheelPidController.setReference(-rpm, ControlType.kVelocity);
    topFlywheelPidController.setReference(rpm, ControlType.kVelocity);

    // Check tolerance
    return (
      topMotor.getEncoder().getVelocity() >= (rpm-Constants.Turret.flyWheelTolerance) && 
      bottomMotor.getEncoder().getVelocity() <= (Constants.Turret.flyWheelTolerance-rpm)
    );
  }
    public void feed() {
    feeder.set(-Constants.Turret.feederSpeed);
  }
      public void stopFeed() {
    feeder.set(0);
  }

  public void setAngle(double angle) {
    targetAngle = angle;
  }
  public void stopShooter() {
    topMotor.set(0);
    bottomMotor.set(0);
  }
}
