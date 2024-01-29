package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
    public CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    public CANSparkFlex rotateMotor = new CANSparkFlex(Constants.Intake.rotateMotorID, MotorType.kBrushless);
    static Intake instance;
    public CANcoder rotateEncoder = new CANcoder(63);
    public positionOptions position = positionOptions.SOURCE;
    
  
    /**
     * @return singleton instance of Intake
     */
    public static Intake getInstance() {
      if (instance == null) {
        instance = new Intake();
      }
  
      return instance;
    }
     /** Creates a new Intake. */
  private Intake() {}

  @Override
  public void periodic() {
  
    switch (position) {
        case SOURCE:
        if(rotateEncoder.getAbsolutePosition().getValueAsDouble() > Constants.Intake.source){
            rotateDown();
        }else{
            rotateUp();
        } 
            break;
        case GROUND:
        if(rotateEncoder.getAbsolutePosition().getValueAsDouble() > Constants.Intake.ground){
            rotateDown();
        }else{
            rotateUp();
        } 
        case DEPOSIT:
        if(rotateEncoder.getAbsolutePosition().getValueAsDouble() > Constants.Intake.deposit){
            rotateDown();
        }else{
            rotateUp();
        } 
    
        default:
            break;
    }
    
  }
    /**
   * Sets the speed of the flywheels
   * @param percentPower [-1, 1]
   */
    public void setIntake() {
    intakeMotor.set(Constants.Intake.intakeMotorPower);
  }
  public void setOutake(){
    intakeMotor.set(-Constants.Intake.intakeMotorPower);
  }
  public enum positionOptions{
    GROUND,
    SOURCE,
    DEPOSIT
    }
 public void rotateUp() {
    rotateMotor.set(Constants.Intake.rotateMotorPower);
  }
public void rotateDown() {
    rotateMotor.set(-Constants.Intake.rotateMotorPower);
  }
public void setPosition(positionOptions position){
    this.position = position;
}
}
