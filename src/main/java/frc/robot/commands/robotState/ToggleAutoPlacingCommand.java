package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleAutoPlacingCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public ToggleAutoPlacingCommand(RobotStateSubsystem robotState) {
    this.robotStateSubsystem = robotState;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAutoPlacing(!robotStateSubsystem.getIsAutoPlacing());
  }
}
