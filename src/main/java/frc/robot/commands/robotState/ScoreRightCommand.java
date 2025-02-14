package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreRightCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ScoreRightCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.setScoreSide(ScoreSide.RIGHT);
    }
}
