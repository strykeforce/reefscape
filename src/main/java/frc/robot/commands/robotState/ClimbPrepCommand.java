package frc.robot.commands.robotState;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimbPrepCommand extends InstantCommand {
    RobotStateSubsystem robotState;
    public ClimbPrepCommand(RobotStateSubsystem robotState){
        this.robotState = robotState;
    }
    @Override
    public void initialize() {
        robotState.toPrepClimb();
    }
}
