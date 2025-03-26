package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class AutonSetPrestageCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private BiscuitSubsystem biscuitSubsystem;

  public AutonSetPrestageCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.biscuitSubsystem = biscuitSubsystem;

    addRequirements(elevatorSubsystem, biscuitSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toAutonPrestage();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() == RobotStates.PRESTAGE;
  }
}
