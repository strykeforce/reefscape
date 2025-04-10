package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class ProtectAlgaeCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public ProtectAlgaeCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toProtectAlgae();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() == RobotStates.PROTECT_ALGAE;
  }
}
