// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.autos.commands;

import frc.robot.commands.MoveIntakeFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;

/** Add your docs here. */
public class clearIntake extends MoveIntakeFirst {
    public clearIntake() {
        super(
            IntakeSubsystem.getInstance(), 
            Shooter.getInstance(), 
            intakePosition.DEPOSIT.get(), 
            shooterPosition.GROUND.get()
        );    
    }
}
