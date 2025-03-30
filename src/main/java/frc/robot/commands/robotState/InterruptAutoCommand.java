package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class InterruptAutoCommand extends Command {
  private RobotStateSubsystem robotState;
  private boolean ejectCoral = false;
  private boolean ejectAlgae = false;
  private RobotStates interruptedState;

  public InterruptAutoCommand(RobotStateSubsystem robotState, CoralSubsystem coralSubsystem) {
    this.robotState = robotState;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    interruptedState = robotState.getInterruptedState();
    if (robotState.getState() == RobotStates.INTERRUPTED) {
      if (interruptedState == RobotStates.BARGE_ALIGN
          || interruptedState == RobotStates.TO_BARGE_ALGAE
          || interruptedState == RobotStates.BARGE_ALGAE) {
        ejectAlgae = true;
        robotState.releaseAlgae();
      } else {
        ejectCoral = true;
        robotState.toPlaceCoral();
      }
    } else {
      ejectCoral = false;
      robotState.toInterrupted();
    }
  }

  @Override
  public boolean isFinished() {
    return !ejectCoral
        || (ejectCoral && !robotState.hasCoral())
        || (ejectAlgae && !robotState.hasAlgae());
  }
}
