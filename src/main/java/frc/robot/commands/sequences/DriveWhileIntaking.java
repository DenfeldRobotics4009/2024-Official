// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.auto.pathing.DriveWithSource;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveShooterFirst;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem.intakePosition;
import frc.robot.subsystems.Shooter.shooterPosition;
import frc.robot.subsystems.SwerveDrive;

public class DriveWhileIntaking extends ParallelCommandGroup {
  /**
   * Lowers the intake to the ground, and sets the shooter
   * to the deposit position.
   */
  public DriveWhileIntaking(Path path, Rotation2d defaultIntakeAngle) {
    super(
      new DriveWithSource(

          new FollowPath(path),

          new Intake(
            IntakeSubsystem.getInstance(), 
            RobotContainer.cam2, 
            defaultIntakeAngle
          ),

          true,

          SwerveDrive.getInstance()
      )
    );
  }
}
