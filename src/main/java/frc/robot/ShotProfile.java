// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class holds a collection of accurate and recorded data points,
 * which detail the distance of our robot to the target, and the height
 * of the shooter when consistent shots were made.
 */
public final class ShotProfile {

    static Translation2d[] data = {
        // x = distance (inches), y = height (joystick value)
        new Translation2d(0, 0),
        new Translation2d(0.95, 0),
        new Translation2d(1.6, -60),
        new Translation2d(2.7, -68),
        new Translation2d(3, -85),
        new Translation2d(3.5, -87),
        new Translation2d(3.8, -88),
        new Translation2d(3.9, -89.5),
        new Translation2d(4, -90),
        new Translation2d(4.3, -93),
        new Translation2d(4.6, -96),
        new Translation2d(5, -100),
        // new Translation2d(1.5, -40),
        // new Translation2d(2.5, -54),
        // new Translation2d(3.7, -90), // Podium line
        // new Translation2d(4.20, -85),
        // new Translation2d(4.40, -86.8),
        // new Translation2d(4.47, -87),
        // new Translation2d(5.1, -89.2),
        // new Translation2d(5.30, -89.6),
        // new Translation2d(5.44, -90),
        // new Translation2d(5.56, -90.45)
        // new Translation2d(0, 0),
        // new Translation2d(33, 0),
        // new Translation2d(76, 0),//-0.35),
        // new Translation2d(84.5,0),// -0.46),
        // new Translation2d(118,0),// -0.54),
        // new Translation2d(126,0),// -0.60),
        // new Translation2d(146,0),// -0.62),
        // new Translation2d(170,0),// -0.64),
        // new Translation2d(175,0),// -0.66),
        // new Translation2d(180,0),// -0.68),
        // new Translation2d(207.5,0),// -0.72),
        // new Translation2d(237,0)// -0.9)
        new Translation2d(8.19, -110)
    };

    /**
     *
     * @param distance in meters
     * @return height of shooter (Degrees from 0)
     */
    public static Optional<Double> getHeightFromDistance(double distance) {
        System.out.println("Distance: " + distance);

        if (distance < data[0].getX()) return Optional.empty(); // Distance is out of bounds

        for (int i = 0; i < data.length-1; i++) {
            if (distance >= data[i].getX() && distance <= data[i+1].getX()) {
                return Optional.of(
                    data[i].interpolate(data[i+1], (distance - data[i].getX()) / (data[i+1].getX() - data[i].getX())).getY()
                        + Constants.Shooter.shotProfileOffset
                );
            }
        }
        return Optional.empty(); // Angle cannot be assumed, distance is out of bounds
    }
}
