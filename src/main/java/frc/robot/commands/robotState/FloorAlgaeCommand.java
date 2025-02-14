package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem; 
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FloorAlgaeCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public FloorAlgaeCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toAlgaeFloorPickup();
    }
}
