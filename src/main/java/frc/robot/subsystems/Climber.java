// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

  private Climber() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftClimber", leftClimberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("rightClimber", rightClimberMotor.getEncoder().getPosition());
  }

  public void moveClimbersUp() {
    setClimber(leftClimberMotor, Constants.Climber.climberMotorPower);
    setClimber(rightClimberMotor, -Constants.Climber.climberMotorPower);
  }

  public void moveClimbersDown() {
    setClimber(leftClimberMotor, -Constants.Climber.climberMotorPower);
    setClimber(rightClimberMotor, Constants.Climber.climberMotorPower);
  }

  public void stop() {
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

  void setClimber(CANSparkMax climberMotor, double speed) {
    if (
      // Check if the climber is going out of bounds
      speed < 0 && leftClimberMotor.getEncoder().getPosition() > Constants.Climber.down || 
      speed > 0 && leftClimberMotor.getEncoder().getPosition() < Constants.Climber.up
    ) {
      leftClimberMotor.set(speed);
    }

    leftClimberMotor.set(0);
  }
}
