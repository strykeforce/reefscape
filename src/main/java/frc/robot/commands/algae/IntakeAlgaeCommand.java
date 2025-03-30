package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class IntakeAlgaeCommand extends InstantCommand {
  private AlgaeSubsystem algaeSubsystem;

  public IntakeAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
  }

  @Override
  public void initialize() {
    algaeSubsystem.intakeAlgae();
  }
}
