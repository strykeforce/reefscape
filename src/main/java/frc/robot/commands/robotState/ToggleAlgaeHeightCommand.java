package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.AlgaeHeight;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleAlgaeHeightCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ToggleAlgaeHeightCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toggleAlgaeHeight(null);
    }
}
