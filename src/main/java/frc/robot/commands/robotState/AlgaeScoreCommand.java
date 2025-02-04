package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeScoreCommand extends InstantCommand {
    RobotStateSubsystem robotState;

    public AlgaeScoreCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
        robotState.releaseAlgae();
    }
}
