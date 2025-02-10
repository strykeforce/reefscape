package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.coral.CoralSubsystem;

public class EnableEjectBeamCommand extends InstantCommand {
  private CoralSubsystem coralSubsystem;
  private boolean enable;

  public EnableEjectBeamCommand(boolean enable, CoralSubsystem coralSubsystem) {
    this.coralSubsystem = coralSubsystem;
    this.enable = enable;
  }

  @Override
  public void initialize() {
    coralSubsystem.enableEjectBeam(enable);
  }
}
