// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.positionOptions;

public class Climber extends SubsystemBase {
    public CANcoder climberEncoder = new CANcoder(63);
    public CANSparkFlex climberMotor = new CANSparkFlex(Constants.Climber.climberMotorID, MotorType.kBrushless);
    public positionOptions position = positionOptions.DOWN;
    static Climber instance;

    public enum positionOptions{
        UP,
        DOWN,
        STOP
    }

    public void rotateUp() {
        climberMotor.set(Constants.Climber.climberMotorPower);
      }
    public void rotateDown() {
        climberMotor.set(-Constants.Climber.climberMotorPower);
      }
    public void setPosition(positionOptions position){
        this.position = position;
    }

  /** Creates a new Climber. */
  public Climber() {}
  
    /**
     * @return singleton instance of Intake
     */
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }

    return instance;
  }  
  @Override
  public void periodic() {

    switch (position) {
        case DOWN:
            if(climberEncoder.getAbsolutePosition().getValueAsDouble() < Constants.Climber.down){
                rotateUp();
            }else{
                rotateDown();
            } 
            break;
        case UP:
            if(climberEncoder.getAbsolutePosition().getValueAsDouble() > Constants.Climber.up){
                rotateDown();
            }else{
                rotateUp();
            } 
            break;
        case STOP:
                climberMotor.set(0);
            break;
        default:
            break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}