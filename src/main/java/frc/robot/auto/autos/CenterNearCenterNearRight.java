package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.pathing.FollowPath;
import frc.robot.auto.pathing.pathObjects.Path;
import frc.robot.auto.pathing.pathObjects.PathPoint;
import frc.robot.auto.paths.centerStart.threePiece.CenterNearCenterNearRightPath;
import frc.robot.auto.util.SetDrivePosition;

public class CenterNearCenterNearRight extends SequentialCommandGroup {
  public CenterNearCenterNearRight(){
    super(
      new FollowPath(
        new Path(
          new PathPoint(
              Constants.Paths.START_CENTER,               // Position (meters)
              new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
              3,    // Speed (m/s)
              new PrintCommand("Shoot")       // Command 
          ),
          new PathPoint(
              new Translation2d(1.5,0),               // Position (meters)
              new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
              2,    // Speed (m/s)
              new PrintCommand("Intake")       // Command 
          ),
          new PathPoint(
              new Translation2d(0.5,-1),               // Position (meters)
              new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
              1,    // Speed (m/s)
              new PrintCommand("Transfer to next")       // Command 
          ),
          new PathPoint(
              new Translation2d(1.5,-2),               // Position (meters)
              new Rotation2d(Math.toRadians(180)),     // Rotation (rad)
              0,    // Speed (m/s)
              new PrintCommand("Intake")       // Command 
          )
        )
      )
    );
  }
}