package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.AlgaeHeight;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class MicAlgaeCommand extends Command {
  private RobotStateSubsystem robotState;
  private boolean hasTriedToPickup = false;
  private boolean safeMoveElevator;

  public MicAlgaeCommand(
      RobotStateSubsystem robotState,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotState = robotState;
    addRequirements(elevatorSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    safeMoveElevator = robotState.safeMoveElevator();
    hasTriedToPickup = false;
    robotState.setAlgaeHeight(AlgaeHeight.HIGH);
    robotState.toAlgaeFloorPickup();
  }

  @Override
  public boolean isFinished() {
    if (robotState.getState() == RobotStates.MIC_ALGAE) {
      hasTriedToPickup = true;
    }
    return !safeMoveElevator || robotState.getState() == RobotStates.STOW && hasTriedToPickup;
  }
}
