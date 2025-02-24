package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;

public class SetScoringLevelCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private ScoringLevel level;

  public SetScoringLevelCommand(RobotStateSubsystem robotStateSubsystem, ScoringLevel level) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.level = level;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setScoringLevel(level);
  }
}
