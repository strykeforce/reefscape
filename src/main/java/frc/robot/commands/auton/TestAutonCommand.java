package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class TestAutonCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private DriveSubsystem driveSubsystem;
  private DriveAutonCommand startPath;
  private RobotStateSubsystem robotStateSubsystem;

  public TestAutonCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem, Pose2d startPose) {

    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    this.startPath = new DriveAutonCommand(driveSubsystem, "LTofetch", true, true, false);

    addCommands(
        new SequentialCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(300.0), startPose),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            startPath));
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
  }
}
