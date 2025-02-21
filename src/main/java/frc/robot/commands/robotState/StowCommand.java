package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class StowCommand extends Command {
  RobotStateSubsystem robotState;

  public StowCommand(
      RobotStateSubsystem robotState,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotState = robotState;
    addRequirements(elevatorSubsystem, coralSubsystem, biscuitSubsystem, algaeSubsystem);
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
