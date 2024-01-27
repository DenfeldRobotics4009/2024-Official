package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controls {

    public final Joystick drive = new Joystick(0);
    public final Joystick steer = new Joystick(1);

    public Controls() {}


    /**
     * Y axis of the drive joystick
     * @return double
     */
    public double getForward() {
        return modifyAxis(-drive.getY(), 0.15);
    }

    /**
     * X axis of the drive joystick
     * @return double
     */
    public double getLateral() {
        return modifyAxis(-drive.getX(), 0.15);
    }

    /**
     * Z axis of the steer joystick
     * @return double
     */
    public double getTurn() {
        return 0 ;//modifyAxis(steer.getZ(), 0.15);
    }

    /**
     * Trigger value of the driver joystick, 
     * indicating prescision mode
     * @return boolean
     */
    public boolean getPrecisionMode() {
        return drive.getTrigger();
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
