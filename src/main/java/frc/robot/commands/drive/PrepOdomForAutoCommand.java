package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class PrepOdomForAutoCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private Rotation2d offset;
  private Pose2d startPose;

  public PrepOdomForAutoCommand(
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      Rotation2d offset,
      Pose2d startPose) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    this.offset = offset;
    this.startPose = startPose;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAuto(true);
    driveSubsystem.prepForAuto(startPose, offset.getDegrees());
  }
}
