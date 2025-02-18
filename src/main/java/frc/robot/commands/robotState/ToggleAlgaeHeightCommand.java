package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleAlgaeHeightCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public ToggleAlgaeHeightCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toggleAlgaeHeight();
  }
}
