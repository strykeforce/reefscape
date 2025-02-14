package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class InteruptAutoCommand extends InstantCommand{
    RobotStateSubsystem robotState;

    public InteruptAutoCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toInterrupted();
    }
}

