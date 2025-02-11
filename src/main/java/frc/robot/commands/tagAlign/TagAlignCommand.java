package frc.robot.commands.tagAlign;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;

public class TagAlignCommand extends Command {
  private TagAlignSubsystem tagAlignSubsystem;

  public TagAlignCommand(TagAlignSubsystem tagAlignSubsystem, DriveSubsystem driveSubsystem) {
    this.tagAlignSubsystem = tagAlignSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    tagAlignSubsystem.start(Alliance.Blue, false);
  }

  @Override
  public boolean isFinished() {
    return tagAlignSubsystem.getState() == TagAlignStates.DONE;
  }
}
