// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
  CANSparkMax rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);

  static Climber instance;

  public static Climber getInstance() {
    if (instance == null) {
      return new Climber();
    }

    return instance;
  }

  private Climber() {
    leftClimberMotor.getEncoder().setPosition(0);
    rightClimberMotor.getEncoder().setPosition(0);

    rightClimberMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.up);
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.down);
    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leftClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-Constants.Climber.up);
    leftClimberMotor.setSoftLimit(SoftLimitDirection.kForward, (float)-Constants.Climber.down);
    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftClimber", leftClimberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("rightClimber", rightClimberMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("leftCurrent", leftClimberMotor.getOutputCurrent());
    SmartDashboard.putNumber("rightCurrent", rightClimberMotor.getOutputCurrent());
  }

  public void moveClimbersUp() {
    setLeftClimber(Constants.Climber.climberMotorPower);
    setRightClimber(Constants.Climber.climberMotorPower);
  }

  public void moveClimbersDown() {
    setLeftClimber(-Constants.Climber.climberMotorPower);
    setRightClimber(-Constants.Climber.climberMotorPower);
  }

  public void stop() {
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

  void setRightClimber(double speed) {
    // if (rightClimberMotor.getEncoder().getPosition() < -Constants.Climber.up && speed > 0) {
    //   rightClimberMotor.set(0);
    //   return;
    // }

    // if (rightClimberMotor.getEncoder().getPosition() > -Constants.Climber.down && speed < 0) {
    //   rightClimberMotor.set(0);
    //   return;
    // }

    rightClimberMotor.set(speed);
  }

  void setLeftClimber(double speed) {
    // if (leftClimberMotor.getEncoder().getPosition() > Constants.Climber.up && speed > 0) {
    //   leftClimberMotor.set(0);
    //   return;
    // }

    // if (leftClimberMotor.getEncoder().getPosition() < Constants.Climber.down && speed < 0) {
    //   leftClimberMotor.set(0);
    //   return;
    // }

    leftClimberMotor.set(-speed);
  }
}
