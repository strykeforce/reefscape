package frc.robot.commands.PathHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pathHandler.PathHandler;

public class KillPathHandlerCommand extends Command {

    PathHandler pathHandler;

    public KillPathHandlerCommand (PathHandler pathHandler) {
        this.pathHandler = pathHandler;
    }

    @Override
    public void execute() {
        pathHandler.killPathHandler();
    }
}
