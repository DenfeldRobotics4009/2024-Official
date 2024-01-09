// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  // Example spark initialization
  // CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);



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
  private Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the flywheels
   * @param percentPower [-1, 1]
   */
  public void setFlyWheelSpeed(double percentPower) {


  }
}
