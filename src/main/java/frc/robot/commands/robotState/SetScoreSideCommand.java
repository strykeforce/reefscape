package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;

public class SetScoreSideCommand extends InstantCommand {
  private RobotStateSubsystem robotState;
  private ScoreSide side;

  public SetScoreSideCommand(RobotStateSubsystem robotState, ScoreSide side) {
    this.robotState = robotState;
    this.side = side;
  }

  @Override
  public void initialize() {
    robotState.setScoreSide(side);
  }
}
