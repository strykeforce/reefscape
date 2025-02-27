package frc.robot.commands.tagAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
import java.util.function.DoubleSupplier;

public class DriveTuningCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private Pose2d start;
  private boolean driving = false;
  private DoubleSupplier pSupplier;

  public DriveTuningCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier pSupplier,
      TagAlignSubsystem tagAlignSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.pSupplier = pSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    start = driveSubsystem.getPoseMeters();
    driving = false;
    driveSubsystem.move(0.5, 0, 0, false);
  }

  @Override
  public void execute() {
    if (!driving
        && driveSubsystem.getPoseMeters().getTranslation().getDistance(start.getTranslation())
            > 1) {
      driving = true;
      tagAlignSubsystem.start(Alliance.Blue, true, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDriving();
  }

  @Override
  public boolean isFinished() {
    return driving && tagAlignSubsystem.getState() == TagAlignStates.DONE;
  }
}
