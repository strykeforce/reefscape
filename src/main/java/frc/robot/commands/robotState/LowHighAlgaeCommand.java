package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.AlgaeHeight;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LowHighAlgaeCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public LowHighAlgaeCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        if(AlgaeHeight.LOW == AlgaeHeight.valueOf(getName())) {
            robotState.setAlgaeHeight(AlgaeHeight.HIGH);
        }
        else{
            robotState.setAlgaeHeight(AlgaeHeight.LOW);
        }
    }
}
