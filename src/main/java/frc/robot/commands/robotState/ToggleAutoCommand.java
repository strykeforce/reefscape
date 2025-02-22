package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ToggleAutoCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public ToggleAutoCommand(RobotStateSubsystem robotState) {
    this.robotStateSubsystem = robotState;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAutoPlacing(!robotStateSubsystem.getIsAutoPlacing());
  }
}
