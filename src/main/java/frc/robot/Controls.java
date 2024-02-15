package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;

public class Controls {

    public final Joystick drive = new Joystick(0);
    public final Joystick steer = new Joystick(1);
    public final Joystick operate = new Joystick(3);

    public final XboxController driveController = new XboxController(2);
    //public final XboxController operateController = new XboxController(4);

    SendableChooser<Integer> driveMode = new SendableChooser<Integer>();
    //SendableChooser<Integer> operateMode = new SendableChooser<Integer>();

    static Controls instance;

    public static Controls getInstance() {
        if (instance == null) {
            instance = new Controls();
        }

        return instance;
    }

    /**
     * Creates a new controls object
     */
    private Controls() {

        driveMode.setDefaultOption("Joystick Driver", 0);
        driveMode.addOption("XBox Driver", 1);

        // operateMode.setDefaultOption("Joystick Operator", 0);
        // operateMode.addOption("XBox Operator", 1);

        SwerveDrive.getInstance().swerveTab.add(driveMode);
        //SwerveDrive.getInstance().swerveTab.add(operateMode);
    }

    /**
     * Y axis of the drive joystick
     * @return double
     */
    public double getForward() {
        switch (driveMode.getSelected()) {
            case 0:
                return modifyAxis(-drive.getY(), 0.15);        
            case 1:
                return modifyAxis(-driveController.getLeftY(), 0.15); 
            default:
                return 0;
        }
    }

    /**
     * X axis of the drive joystick
     * @return double
     */
    public double getLateral() {
        switch (driveMode.getSelected()) {
            case 0:
                return modifyAxis(-drive.getX(), 0.15);        
            case 1:
                return modifyAxis(-driveController.getLeftX(), 0.15); 
            default:
                return 0;
        }
    }

    /**
     * Z axis of the steer joystick
     * @return double
     */
    public double getTurn() {
        switch (driveMode.getSelected()) {
            case 0:
                return modifyAxis(drive.getZ(), 0.15);        
            case 1:
                return modifyAxis(driveController.getRightX(), 0.15); 
            default:
                return 0;
        }
    }

    /**
     * Trigger value of the driver joystick, 
     * indicating precision mode
     * @return boolean
     */
    public boolean getPrecisionMode() {
        switch (driveMode.getSelected()) {
            case 0:
                return drive.getTrigger();        
            case 1:
                return !(driveController.getRightTriggerAxis() > 0.1); 
            default:
                return false;
        }
    }

    public int getPOV() {
        switch (driveMode.getSelected()) {
            case 0:
                return steer.getPOV();        
            case 1:
                if (driveController.getLeftStickButton()) return 0;
                if (driveController.getRightStickButton()) return 180; 
                return driveController.getPOV(); 
            default:
                return -1;
        }
    }

    /**
     * @param id button id
     * @return JoystickButton on drive joystick
     */
    public JoystickButton getDriverButton(int id) {
        return new JoystickButton(drive, id);
    }

    /**
     * @param id button id
     * @return JoystickButton on steer joystick
     */
    public JoystickButton getSteerButton(int id) {
        return new JoystickButton(steer, id);
    }

    /**
     * @param id button id
     * @return JoystickButton on operate joystick
     */
    public JoystickButton getOperatorButton(int id) {
        return new JoystickButton(operate, id);
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
            } else {
            return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value, double deadband) {
        // Deadband
        value = deadband(value, deadband);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
