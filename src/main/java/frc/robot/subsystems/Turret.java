// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  // Example spark initialization
  // CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);

  private CANSparkFlex topMotor = new CANSparkFlex(Constants.Turret.topMotorID, MotorType.kBrushless);
  private CANSparkFlex bottomMotor = new CANSparkFlex(Constants.Turret.bottomMotorID, MotorType.kBrushless);
  private CANSparkMax feeder = new CANSparkMax(Constants.Turret.feederMotorID, MotorType.kBrushless);
  private CANSparkMax aim = new CANSparkMax(Constants.Turret.aimMotorID, MotorType.kBrushless);
  private PIDController aimPIDController = new PIDController(.1, .1, .01);
  private PIDController flywheelPIDController = new PIDController(.1, 0.1, 0.01);
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
  }

  @Override
  public void periodic() {
    double speed = aimPIDController.calculate(aim.getEncoder().getPosition()*2*Math.PI, targetAngle);
    MathUtil.clamp(speed, -1, 1);
    aim.set(speed);
    // This method will be called once per scheduler run

  }

  public boolean atTargetAngle() {
    return aimPIDController.atSetpoint();
  }

  /**
   * Sets the speed of the flywheels
   * @param percentPower [-1, 1]
   */
  public boolean setFlyWheelSpeed(double rpm) {
    double speed = flywheelPIDController.calculate(aim.getEncoder().getVelocity(), rpm);
    MathUtil.clamp(speed, -1, 1);
    topMotor.set(speed);
    bottomMotor.set(-speed);
    return (topMotor.getEncoder().getVelocity()>=rpm) && (bottomMotor.getEncoder().getVelocity()<=rpm);
  }
    public void feed() {
    feeder.set(Constants.Turret.feederSpeed);
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
