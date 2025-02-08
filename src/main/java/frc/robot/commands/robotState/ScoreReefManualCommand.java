package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ScoreReefManualCommand extends Command {
  public RobotStateSubsystem robotStateSubsystem;
  public RobotStates robotState;

  public ScoreReefManualCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotState = robotStateSubsystem.getState();
    robotStateSubsystem.setIsAuto(false);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    robotStateSubsystem.toPrepCoral();
  }

  @Override
  public boolean isFinished() {
    if (robotState == RobotStates.PRESTAGE || robotState == RobotStates.STOW) {
      return robotStateSubsystem.getState() == RobotStates.REEF_ALIGN_CORAL;
    }
    if (robotState == RobotStates.REEF_ALIGN_CORAL) {
      return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD
          || robotStateSubsystem.getState() == RobotStates.LOADING_CORAL;
    }
    return false;
  }
}
