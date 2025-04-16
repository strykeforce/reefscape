package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.AlgaeHeight;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ForceLowFloorAlgaeCommand extends Command {
  private RobotStateSubsystem robotState;
  boolean hasTriedToPickup = false;
  boolean notSafeElevator = false;

  public ForceLowFloorAlgaeCommand(
      RobotStateSubsystem robotState,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotState = robotState;
    addRequirements(elevatorSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    hasTriedToPickup = false;
    notSafeElevator = !robotState.safeMoveElevator();
    robotState.setAlgaeHeight(AlgaeHeight.LOW);
    robotState.toAlgaeFloorPickup();
  }

  @Override
  public boolean isFinished() {
    if (robotState.getState() == RobotStates.FLOOR_ALGAE
        || robotState.getState() == RobotStates.MIC_ALGAE) {
      hasTriedToPickup = true;
    }
    return (robotState.getState() == RobotStates.STOW && hasTriedToPickup) || notSafeElevator;
  }
}
