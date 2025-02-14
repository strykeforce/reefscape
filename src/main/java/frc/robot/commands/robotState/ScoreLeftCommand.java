package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreLeftCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ScoreLeftCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.setScoreSide(ScoreSide.LEFT);
    }
}
