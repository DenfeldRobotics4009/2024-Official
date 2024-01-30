// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(60, MotorType.kBrushless);
  static IntakeSubsystem instance;

  public static IntakeSubsystem GetInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }

    return instance;
  }

  /** Creates a new Intake. */
  private IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Intake at constant speed
   */
  public void intake() {
    intakeMotor.set(1);
  }

  /**
   * Outtake at constant speed
   */
  public void outtake() {
    intakeMotor.set(-1);
  }
}
