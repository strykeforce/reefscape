package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SpinUpWheelsCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;

  public SpinUpWheelsCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.setAzimuthVel(0.2);
  }
}