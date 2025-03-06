package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class InterruptAutoCommand extends Command {
  private RobotStateSubsystem robotState;
  private boolean ejectCoral = false;

  public InterruptAutoCommand(RobotStateSubsystem robotState, CoralSubsystem coralSubsystem) {
    this.robotState = robotState;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    if (robotState.getState() == RobotStates.INTERRUPTED) {
      ejectCoral = true;
      robotState.toPlaceCoral();
    } else {
      ejectCoral = false;
      robotState.toInterrupted();
    }
  }

  @Override
  public boolean isFinished() {
    return !ejectCoral || !robotState.hasCoral();
  }
}
