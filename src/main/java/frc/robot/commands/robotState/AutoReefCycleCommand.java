package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;

public class AutoReefCycleCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private DriveSubsystem driveSubsystem;
  private boolean scoringCoral;

  public AutoReefCycleCommand(
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem,
      DriveSubsystem driveSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      AlgaeSubsystem algaeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.driveSubsystem = driveSubsystem;

    addRequirements(elevatorSubsystem, coralSubsystem, biscuitSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setIgnoreSticks(true);
    robotStateSubsystem.toReefAlign();
    scoringCoral =
        robotStateSubsystem.hasCoral()
            && !(robotStateSubsystem.getGetAlgaeOnCycle()
                && robotStateSubsystem.getScoreSide() == ScoreSide.RIGHT);
  }

  @Override
  public void end(boolean interrupted) {
    tagAlignSubsystem.terminate();
  }

  @Override
  public boolean isFinished() {
    return scoringCoral && !robotStateSubsystem.hasCoral()
        || !scoringCoral && robotStateSubsystem.hasAlgae();
  }
}
