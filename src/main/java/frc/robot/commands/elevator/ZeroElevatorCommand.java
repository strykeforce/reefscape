package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;

public class ZeroElevatorCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;

  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.zero();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getState() == ElevatorStates.ZEROED;
  }
}
