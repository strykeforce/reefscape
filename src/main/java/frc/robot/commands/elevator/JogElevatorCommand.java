package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class JogElevatorCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private Angle positionChange;

  public JogElevatorCommand(ElevatorSubsystem elevatorSubsystem, Angle positionChange) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.positionChange = positionChange;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPosition(elevatorSubsystem.getPosition().plus(positionChange));
  }

  @Override
  public void execute() {
    elevatorSubsystem.setPosition(elevatorSubsystem.getPosition().plus(positionChange));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
