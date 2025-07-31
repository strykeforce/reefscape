package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.AlgaeHeight;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoForceBargeCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private boolean hasTriedToPickup = false;
  private RobotStates startState;
  private boolean startingElevatorFinished;
  private boolean hasEjectedToBarge;
  private boolean safeMoveElevator;
  private VisionSubsystem visionSubsystem;

  public AutoForceBargeCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem,
      VisionSubsystem visionSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(elevatorSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    safeMoveElevator = robotStateSubsystem.safeMoveElevator();
    hasEjectedToBarge = false;
    startState = robotStateSubsystem.getState();
    startingElevatorFinished = elevatorSubsystem.isFinished();
    robotStateSubsystem.setAlgaeHeight(AlgaeHeight.HIGH);
    if (startState == RobotStates.BARGE_ALGAE && !robotStateSubsystem.getIsAutoPlacing())
      robotStateSubsystem.releaseAlgae();
    else robotStateSubsystem.toScoreAlgae();

    visionSubsystem.setIsAuto(false);
  }

  @Override
  public boolean isFinished() {
    if (!safeMoveElevator) {
      return true;
    }
    if (startState == RobotStates.PROCESSOR_ALGAE
        || startState == RobotStates.BARGE_ALGAE
        || hasEjectedToBarge) {
      return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD
          || robotStateSubsystem.getState() == RobotStates.STOW
          || !startingElevatorFinished;
    } else if (robotStateSubsystem.getState() == RobotStates.BARGE_ALGAE
        && robotStateSubsystem.getIsAutoPlacing()) {
      hasEjectedToBarge = true;
      return false;
    } else {
      return robotStateSubsystem.getState() == RobotStates.PROCESSOR_ALGAE
          || (robotStateSubsystem.getState() == RobotStates.BARGE_ALGAE
              && !robotStateSubsystem.getIsAutoPlacing())
      /*|| !robotStateSubsystem.getIsBargeSafe() */ ;
    }
  }

  @Override
  public void end(boolean interrupted) {
    visionSubsystem.setIsAuto(true);
  }
}
