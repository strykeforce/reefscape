package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GetAlgaeCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public GetAlgaeCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
        public void initialize() {
            if(robotState.getGetAlgaeOnCycle() == true) {
                robotState.setGetAlgaeOnCycle(false);
            }
            else{
                robotState.setGetAlgaeOnCycle(true);
            }
        }
    }
