package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeArm extends SubsystemBase {
    public CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    public CANSparkFlex rotateMotor = new CANSparkFlex(Constants.Intake.rotateMotorID, MotorType.kBrushless);
    static IntakeArm instance;
    public CANcoder rotateEncoder = new CANcoder(63);
    public positionOptions intakePosition = positionOptions.STARTING;
    public PIDController intakePIDController = new PIDController(.1, 0.1, 0.01);
    
  
    /**
     * @return singleton instance of Intake
     */
    public static IntakeArm getInstance() {
      if (instance == null) {
        instance = new IntakeArm();
      }
  
      return instance;
    }
     /** Creates a new Intake. */
  private IntakeArm() {}

  @Override
  public void periodic() {
    double currentIntakePosition = rotateEncoder.getAbsolutePosition().getValueAsDouble();
    intakePIDController.setSetpoint(intakePosition.position);
    intakeMotor.set(intakePIDController.calculate(currentIntakePosition));
  }

  public void setIntake() {
    intakeMotor.set(Constants.Intake.intakeMotorPower);
  }
  public void setOuttake(){
    intakeMotor.set(-Constants.Intake.intakeMotorPower);
  }
  public void stop() {
    intakeMotor.set(0);
  }


  public enum positionOptions{
    GROUND (Constants.Intake.ground),
    DEPOSIT (Constants.Intake.deposit),
    STARTING (0);
    public double position;
    private positionOptions(double position) {
      this.position = position;
    }
  }

  public void setPosition(positionOptions position){
    intakePosition = position;
  }
}
