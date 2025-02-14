package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreAlgaeCommand extends InstantCommand {
    RobotStateSubsystem robotState;

    public ScoreAlgaeCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
        robotState.toScoreAlgae();
    }
}
