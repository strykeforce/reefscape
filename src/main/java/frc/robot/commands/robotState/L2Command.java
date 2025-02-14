package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class L2Command extends InstantCommand {
    RobotStateSubsystem robotState;
    public L2Command(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.setScoringLevel(ScoringLevel.L2);
    }
}
