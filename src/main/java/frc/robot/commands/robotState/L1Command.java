package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class L1Command extends InstantCommand {
    RobotStateSubsystem robotState;
    public L1Command(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.setScoringLevel(ScoringLevel.L1);
    }
}
