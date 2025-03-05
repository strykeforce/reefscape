package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ClimbCommand extends Command {
  RobotStateSubsystem robotState;
  ClimbSubsystem climbSubsystem;

  public ClimbCommand(RobotStateSubsystem robotState, ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.robotState = robotState;
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    robotState.toClimb();
  }

  @Override
  public boolean isFinished() {
    return !climbSubsystem.isClimbFinished();
  }
}
