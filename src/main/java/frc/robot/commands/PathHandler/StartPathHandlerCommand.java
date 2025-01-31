package frc.robot.commands.PathHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pathHandler.PathHandler;

public class StartPathHandlerCommand extends Command {

    PathHandler pathHandler;

    public StartPathHandlerCommand (PathHandler pathHandler) {
        this.pathHandler = pathHandler;
    }

    @Override
    public void execute() {
        pathHandler.startPathHandler();
    }

    @Override
    public boolean isFinished() {
        return pathHandler.isFinished();
    }
}
