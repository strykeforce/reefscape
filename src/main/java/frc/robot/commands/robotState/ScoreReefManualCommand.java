package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ScoreReefManualCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private RobotStates startingRobotState;
  private boolean startingElevatorFinished;

  public ScoreReefManualCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem, coralSubsystem);
  }

  @Override
  public void initialize() {
    startingRobotState = robotStateSubsystem.getState();
    startingElevatorFinished = elevatorSubsystem.isFinished();
    robotStateSubsystem.setIsAuto(false);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    robotStateSubsystem.toPrepCoral();
  }

  @Override
  public boolean isFinished() {
    if (startingRobotState == RobotStates.PRESTAGE || startingRobotState == RobotStates.STOW) {
      return robotStateSubsystem.getState() == RobotStates.REEF_ALIGN_CORAL;
    }
    if (startingRobotState == RobotStates.REEF_ALIGN_CORAL) {
      return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD
          || robotStateSubsystem.getState() == RobotStates.LOADING_CORAL
          || !startingElevatorFinished;
    }
    return false;
  }
}
