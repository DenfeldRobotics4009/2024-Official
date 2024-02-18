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
        new Translation2d(33, 0),
        new Translation2d(38, 0),
        new Translation2d(76, -0.35),
        new Translation2d(84.5, -0.46),
        new Translation2d(118, -0.54),
        new Translation2d(126, -0.60),
        new Translation2d(146, -0.62),
        new Translation2d(170, -0.64),
        new Translation2d(175, -0.66),
        new Translation2d(180, -0.68),
        new Translation2d(207.5, -0.72),
        new Translation2d(237, -0.9)
    };

    /**
     *
     * @param distance in meters
     * @return height of shooter (Degrees from 0)
     */
    public static Optional<Double> getHeightFromDistance(double distance) {
        distance = Utils.metersToInches(distance);
        System.out.println("Distance: " + distance);

        if (distance < data[0].getX()) return Optional.empty(); // Distance is out of bounds

        for (int i = 0; i < data.length-1; i++) {
            if (distance >= data[i].getX() && distance <= data[i+1].getX()) {
                return Optional.of(
                    -data[i].interpolate(data[i+1], distance).getY() * Constants.Shooter.aimRangeFrom0
                );
            }
        }
        return Optional.empty(); // Angle cannot be assumed, distance is out of bounds
    }
}
