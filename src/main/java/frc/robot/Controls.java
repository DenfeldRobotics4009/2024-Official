package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controls {

    public final XboxController driveController = new XboxController(0);
    
    static Controls instance;

    public static Controls GetInstance() {
        if (instance == null) {
            instance = new Controls();
        }

        return instance;
    }

    /**
     * Creates a new controls object
     */
    private Controls() {}

    /**
     * Y axis of the drive joystick
     * @return double
     */
    public double getForward() {
        return modifyAxis(-driveController.getLeftY(), 0.15);
    }

    /**
     * X axis of the drive joystick
     * @return double
     */
    public double getLateral() {
        return modifyAxis(-driveController.getLeftX(), 0.15);
    }

    /**
     * Z axis of the steer joystick
     * @return double
     */
    public double getTurn() {
        return modifyAxis(driveController.getRightX(), 0.15);
    }

    /**
     * Trigger value of the driver joystick, 
     * indicating prescision mode
     * @return boolean
     */
    public boolean getPrecisionMode() {
        return driveController.getRightTriggerAxis() > 0.1;
    }

    /**
     * @param id button id
     * @return JoystickButton on drive joystick
     */
    public JoystickButton getDriverButton(int id) {
        return new JoystickButton(driveController, id);
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
