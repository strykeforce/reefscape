package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ClimbPrepCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public ClimbPrepCommand(
      RobotStateSubsystem robotState,
      ClimbSubsystem climbSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitSubsystem biscuitSubsystem) {
    addRequirements(climbSubsystem, elevatorSubsystem, biscuitSubsystem);
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toPrepClimb();
  }
}
