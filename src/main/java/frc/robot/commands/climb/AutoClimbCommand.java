package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbAlignSubsystem;
import frc.robot.subsystems.climb.ClimbAlignSubsystem.ClimbAlignStates;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class AutoClimbCommand extends Command {
  private ClimbSubsystem climbSubsystem;
  private ClimbAlignSubsystem climbAlignSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public AutoClimbCommand(
      DriveSubsystem driveSubsystem,
      ClimbSubsystem climbSubsystem,
      ClimbAlignSubsystem climbAlignSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    this.climbAlignSubsystem = climbAlignSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem, driveSubsystem);
  }

  @Override
  public void initialize() {
    climbAlignSubsystem.start(robotStateSubsystem.getAllianceColor());
  }

  @Override
  public void end(boolean interrupted) {
    climbAlignSubsystem.terminate();
  }

  @Override
  public boolean isFinished() {
    return climbAlignSubsystem.getState() == ClimbAlignStates.DONE
        && climbSubsystem.getState() == ClimbState.CLIMBED;
  }
}
