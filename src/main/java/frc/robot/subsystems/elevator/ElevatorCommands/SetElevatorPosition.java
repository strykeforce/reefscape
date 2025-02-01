package frc.robot.subsystems.elevator.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Angle;

public class SetElevatorPosition extends Command{
    
    private ElevatorSubsystem elevatorSubsystem;
    private Angle position;

    public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, Angle position) {
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
