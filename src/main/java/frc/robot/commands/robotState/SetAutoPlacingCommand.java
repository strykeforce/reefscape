package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class SetAutoPlacingCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private boolean isAutoPlacing;

  public SetAutoPlacingCommand(RobotStateSubsystem robotState, boolean isAutoPlacing) {
    this.robotStateSubsystem = robotState;
    this.isAutoPlacing = isAutoPlacing;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAutoPlacing(isAutoPlacing);
  }
}
