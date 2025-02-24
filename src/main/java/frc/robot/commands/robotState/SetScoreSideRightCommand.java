package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;

public class SetScoreSideRightCommand extends InstantCommand {
  RobotStateSubsystem robotState;

  public SetScoreSideRightCommand(RobotStateSubsystem robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.setScoreSide(ScoreSide.RIGHT);
  }
}
