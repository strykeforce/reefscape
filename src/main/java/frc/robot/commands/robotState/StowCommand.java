package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class StowCommand extends Command {
  RobotStateSubsystem robotState;

  public StowCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toStow();
  }

  @Override
  public boolean isFinished() {
    return robotState.getState() == RobotStates.STOW;
  }
}
