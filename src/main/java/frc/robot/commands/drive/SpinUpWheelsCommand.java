package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SpinUpWheelsCommand extends Command {
  private DriveSubsystem driveSubsystem;

  public SpinUpWheelsCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setAzimuthVel(0.2);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
