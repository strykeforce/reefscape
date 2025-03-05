package frc.robot.commands.climb;

import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class PrepClimb extends Command {

  RobotStateSubsystem robotStateSubsystem;
  ClimbSubsystem climbSubsystem;

  public PrepClimb(ClimbSubsystem climbSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }
  
  @Override
  public void initialize() {
      robotStateSubsystem.toClimb();
  }

  @Override
  public boolean isFinished() {
    return (robotStateSubsystem.getState() != RobotStates.PREP_CLIMB);
  }

}
