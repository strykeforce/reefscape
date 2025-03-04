package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ClimbPrepCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public ClimbPrepCommand(RobotStateSubsystem robotState) {
    addRequirements(robotState);
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toPrepClimb();
  }
}
