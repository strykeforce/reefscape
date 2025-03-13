package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class PlaceCoralAutonCommand extends Command {
  private RobotStateSubsystem robotStateSubsystem;
  private CoralSubsystem coralSubsystem;

  public PlaceCoralAutonCommand(
      RobotStateSubsystem robotStateSubsystem, CoralSubsystem coralSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.coralSubsystem = coralSubsystem;
    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toPlaceCoralAuto();
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.toFunnelLoad();
  }

  @Override
  public boolean isFinished() {
    return !coralSubsystem.hasCoral();
  }
}
