package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class TestAutonCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private DriveSubsystem driveSubsystem;
  private DriveAutonCommand startPath;
  private RobotStateSubsystem robotStateSubsystem;

  public TestAutonCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem) {

    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    this.startPath = new DriveAutonCommand(driveSubsystem, "FiveMeterTestPath", true, true, false);

    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new PrepOdomForAutoCommand(
                    robotStateSubsystem, driveSubsystem, new Rotation2d(), new Pose2d()),
                new ZeroElevatorCommand(elevatorSubsystem)),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            startPath));
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
  }
}
