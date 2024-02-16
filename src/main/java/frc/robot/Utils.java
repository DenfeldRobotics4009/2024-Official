// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Utils {

    public static double inchesToMeters(double inches) {
        return inches / Constants.inchesInMeter;
    }

    public static double metersToInches(double meters) {
        return meters * Constants.inchesInMeter;
    }
}
