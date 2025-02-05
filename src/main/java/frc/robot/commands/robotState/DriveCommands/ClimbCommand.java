package frc.robot.commands.robotState.DriveCommands;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimbCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ClimbCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toClimb();
    }
}