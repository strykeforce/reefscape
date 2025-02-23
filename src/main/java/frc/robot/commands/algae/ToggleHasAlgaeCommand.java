package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;

public class ToggleHasAlgaeCommand extends InstantCommand {
  private AlgaeSubsystem algaeSubsystem;

  public ToggleHasAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
  }

  @Override
  public void initialize() {
    algaeSubsystem.setState(
        algaeSubsystem.getState() == AlgaeStates.EMPTY ? AlgaeStates.HAS_ALGAE : AlgaeStates.EMPTY);
  }
}
