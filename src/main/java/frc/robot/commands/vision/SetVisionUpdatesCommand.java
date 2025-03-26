package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SetVisionUpdatesCommand extends InstantCommand {
  VisionSubsystem visionSubsystem;
  private boolean enable;

  public SetVisionUpdatesCommand(VisionSubsystem visionSubsystem, boolean enable) {
    this.visionSubsystem = visionSubsystem;
    this.enable = enable;
  }

  @Override
  public void initialize() {
    visionSubsystem.setVisionUpdating(enable);
  }
}
