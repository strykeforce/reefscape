package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class GetAlgaeCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public GetAlgaeCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    if (robotState.getGetAlgaeOnCycle() == true) {
      robotState.setGetAlgaeOnCycle(false);
    } else {
      robotState.setGetAlgaeOnCycle(true);
    }
  }
}
