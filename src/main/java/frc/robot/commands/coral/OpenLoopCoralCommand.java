package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.coral.CoralSubsystem;

public class OpenLoopCoralCommand extends InstantCommand {
  private CoralSubsystem coralSubsystem;
  private double pct;

  public OpenLoopCoralCommand(CoralSubsystem coralSubsystem, double pct) {
    this.coralSubsystem = coralSubsystem;
    this.pct = pct;
  }

  @Override
  public void initialize() {
    coralSubsystem.setPct(pct);
  }
}
