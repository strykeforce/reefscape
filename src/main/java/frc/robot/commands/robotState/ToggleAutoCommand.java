package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleAutoCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ToggleAutoCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        if (robotState.getIsAuto() == false){
            robotState.setIsAuto(true);
        }
        else{
            robotState.setIsAuto(false);
        }
    }
}
