package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class HPAlgaeCommand extends Command {
  private RobotStateSubsystem robotState;
  private boolean hasTriedToGrab = false;
  private boolean safeMoveElevator;

  public HPAlgaeCommand(
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
    hasTriedToGrab = false;
    robotState.toHpAlgae();
  }

  @Override
  public boolean isFinished() {
    if (robotState.getState() == RobotStates.HP_ALGAE) {
      hasTriedToGrab = true;
    }
    return !safeMoveElevator || robotState.getState() == RobotStates.STOW && hasTriedToGrab;
  }
}
