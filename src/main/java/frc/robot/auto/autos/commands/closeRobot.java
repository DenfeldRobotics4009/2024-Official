// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.autos.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

/** Add your docs here. */
public class closeRobot extends MoveShooterFirst{
    public closeRobot() {
        super(
            IntakeSubsystem.getInstance(),
            Shooter.getInstance(),
            intakePosition.STARTING.get(),
            shooterPosition.GROUND.get()
        );
    }

}
