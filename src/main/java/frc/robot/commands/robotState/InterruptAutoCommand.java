package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class InterruptAutoCommand extends InstantCommand {
  private RobotStateSubsystem robotState;

  public InterruptAutoCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.toInterrupted();
  }
}
