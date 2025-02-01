package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorPositionCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private Angle position;

  public SetElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, Angle position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPosition(position);
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isFinished();
  }
}
