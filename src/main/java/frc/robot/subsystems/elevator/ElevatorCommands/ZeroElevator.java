package frc.robot.subsystems.elevator.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ZeroElevator extends Command{

    private ElevatorSubsystem elevatorSubsystem;
    
    public ZeroElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }
    
    @Override
    public void initialize() {
        elevatorSubsystem.zero();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isFinished();//this is probably different than setposition
  }
}
