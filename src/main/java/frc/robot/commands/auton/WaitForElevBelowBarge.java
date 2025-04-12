package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class WaitForElevBelowBarge extends Command {
  private ElevatorSubsystem elevatorSubsystem;

  public WaitForElevBelowBarge(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getPosition().in(Rotations)
        < ElevatorConstants.kUnderBargeSafeThreshold.in(Rotations);
  }
}
