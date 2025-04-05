package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class EjectAlgaeCommand extends InstantCommand {
  private AlgaeSubsystem algaeSubsystem;

  public EjectAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  @Override
  public void initialize() {
    algaeSubsystem.scoreProcessor();
  }
}
