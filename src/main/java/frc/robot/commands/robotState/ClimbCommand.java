package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ClimbCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public ClimbCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toClimb();
  }
}
