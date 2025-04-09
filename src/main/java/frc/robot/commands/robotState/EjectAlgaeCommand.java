package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class EjectAlgaeCommand extends InstantCommand {
  private AlgaeSubsystem algaeSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public EjectAlgaeCommand(AlgaeSubsystem algaeSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(algaeSubsystem);
  }

  public void initialize() {
    robotStateSubsystem.releaseAlgae();
  }
}
