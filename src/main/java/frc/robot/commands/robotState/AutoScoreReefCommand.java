package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;

public class AutoScoreReefCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;

  public AutoScoreReefCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      DriveSubsystem driveSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(elevatorSubsystem, coralSubsystem, driveSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setIsAuto(false);
    robotStateSubsystem.setAutoPlacing(true);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    robotStateSubsystem.setScoreSide(ScoreSide.LEFT);
    robotStateSubsystem.toReefAlign();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getState() == RobotStates.FUNNEL_LOAD;
  }
}
