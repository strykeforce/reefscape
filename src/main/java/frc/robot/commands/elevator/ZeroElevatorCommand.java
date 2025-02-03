package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;

  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void initialize() {
    elevatorSubsystem.zero();
  }
}
