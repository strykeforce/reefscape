package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleGetAlgaeCommand extends InstantCommand {
  RobotStateSubsystem robotStateSubsystem;

  public ToggleGetAlgaeCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toggleGetAlgaeOnCycle();
  }
}
