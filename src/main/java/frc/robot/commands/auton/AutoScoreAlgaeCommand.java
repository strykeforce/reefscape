package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class AutoScoreAlgaeCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private RobotStates startState;
  private boolean startingElevatorFinished;
  private boolean hasEjectedToBarge;

  public AutoScoreAlgaeCommand(
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
    robotStateSubsystem.releaseAlgae();
  }

  @Override
  public boolean isFinished() {
    return !robotStateSubsystem.hasAlgae();
  }
}
