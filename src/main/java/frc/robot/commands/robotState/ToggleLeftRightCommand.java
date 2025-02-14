package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleLeftRightCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ToggleLeftRightCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        if (ScoreSide.valueOf(getName()) == ScoreSide.LEFT){
            robotState.setScoreSide(ScoreSide.RIGHT);
        }
        else{
            robotState.setScoreSide(ScoreSide.LEFT);
        }
    }
}
