package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class HoldElevatorCommand extends InstantCommand {
  private ElevatorSubsystem elevatorSubsystem;

  public HoldElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPosition(elevatorSubsystem.getPosition());
  }
}
