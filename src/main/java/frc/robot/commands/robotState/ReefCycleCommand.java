package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ReefCycleCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private DriveSubsystem driveSubsystem;
  private RobotStates startingRobotState;
  private boolean startingElevatorFinished;
  private boolean isAutoPlacing;

  public ReefCycleCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem, coralSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    startingRobotState = robotStateSubsystem.getState();
    startingElevatorFinished = elevatorSubsystem.isFinished();
    isAutoPlacing = robotStateSubsystem.getIsAutoPlacing();

    robotStateSubsystem.setIsAutoPlacing(false);

    robotStateSubsystem.toPrepCoral();
  }

  @Override
  public boolean isFinished() {

    if (startingRobotState == RobotStates.PRESTAGE || startingRobotState == RobotStates.STOW) {
      return robotStateSubsystem.getState() == RobotStates.REEF_ALIGN_CORAL
          || (!robotStateSubsystem.hasCoral() && !robotStateSubsystem.getGetAlgaeOnCycle());
    }
    if (startingRobotState == RobotStates.REEF_ALIGN_CORAL) {
      return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD
          || robotStateSubsystem.getState() == RobotStates.LOADING_CORAL
          || robotStateSubsystem.getState() == RobotStates.TO_ALGAE_CORAL_LOAD
          || robotStateSubsystem.getState() == RobotStates.ALGAE_CORAL_LOAD
          || !startingElevatorFinished;
    }

    return false;
  }
}
