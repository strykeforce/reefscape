package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class OpenLoopAlgaeCommand extends InstantCommand {
  private AlgaeSubsystem algaeSubsystem;
  private double pct;

  public OpenLoopAlgaeCommand(AlgaeSubsystem algaeSubsystem, double pct) {
    this.algaeSubsystem = algaeSubsystem;
    this.pct = pct;
  }

  @Override
  public void initialize() {
    algaeSubsystem.setPct(pct);
  }
}
