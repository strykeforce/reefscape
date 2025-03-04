package frc.robot.commands.pathhHandler;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pathHandler.PathHandler;

public class KillPathHandlerCommand extends InstantCommand {

  PathHandler pathHandler;

  public KillPathHandlerCommand(PathHandler pathHandler) {
    this.pathHandler = pathHandler;
  }

  @Override
  public void initialize() {
    pathHandler.killPathHandler();
  }
}
