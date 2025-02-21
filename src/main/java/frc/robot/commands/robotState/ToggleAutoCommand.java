package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleAutoCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public ToggleAutoCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    if (robotState.getAutoPlaceOnCycle() == false) {
      robotState.setAutoPlaceOnCycle(true);
    } else {
      robotState.setAutoPlaceOnCycle(false);
    }
  }
}
