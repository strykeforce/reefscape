package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class StowCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public StowCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toStow();
    }
}
