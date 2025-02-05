package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoVsManualCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public AutoVsManualCommand(RobotStateSubsystem robotState){
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
