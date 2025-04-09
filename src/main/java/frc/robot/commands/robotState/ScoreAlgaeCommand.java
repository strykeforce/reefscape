package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ScoreAlgaeCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private RobotStates startState;
  private boolean startingElevatorFinished;
  private boolean hasEjectedToBarge;

  public ScoreAlgaeCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    hasEjectedToBarge = false;
    startState = robotStateSubsystem.getState();
    startingElevatorFinished = elevatorSubsystem.isFinished();
    if (startState == RobotStates.PROCESSOR_ALGAE || startState == RobotStates.BARGE_ALGAE) {
      robotStateSubsystem.releaseAlgae();
    } else {
      robotStateSubsystem.toScoreAlgae();
    }
  }

  @Override
  public boolean isFinished() {
    if (startState == RobotStates.PROCESSOR_ALGAE
        || startState == RobotStates.BARGE_ALGAE
        || hasEjectedToBarge) {
      return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD
          || robotStateSubsystem.getState() == RobotStates.STOW
          || !startingElevatorFinished;
    } else if (robotStateSubsystem.getState() == RobotStates.BARGE_ALGAE
        && robotStateSubsystem.getIsAutoPlacing()) {
      hasEjectedToBarge = true;
    } else {
      return robotStateSubsystem.getState() == RobotStates.PROCESSOR_ALGAE
          || (robotStateSubsystem.getState() == RobotStates.BARGE_ALGAE
              && !robotStateSubsystem.getIsAutoPlacing())
          || !robotStateSubsystem.getIsBargeSafe();
    }
    return false;
  }
}
