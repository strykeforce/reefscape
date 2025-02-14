package frc.robot.commands.robotState.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ReefCycleComand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ReefCycleComand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toReefAlign();
    }
}
