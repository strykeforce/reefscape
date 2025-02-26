package frc.robot.commands.pathHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pathHandler.PathHandler;
import java.util.List;

public class StartPathHandlerCommand extends Command {
  private PathHandler pathHandler;

  public StartPathHandlerCommand(
      PathHandler pathHandler,
      String[][] pathNames,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      Character startNode,
      boolean mirrorToProcessor) {
    pathHandler.setPathNames(pathNames);
    pathHandler.setNodeNames(NodeNames);
    pathHandler.setNodeLevels(NodeLevels);
    pathHandler.setStartNode(startNode);
    pathHandler.setMirrorToProcessor(mirrorToProcessor);
  }

  public StartPathHandlerCommand(PathHandler pathHandler) {
    this.pathHandler = pathHandler;
  }

  @Override
  public void initialize() {
    pathHandler.startPathHandler();
  }

  @Override
  public boolean isFinished() {
    return pathHandler.isFinished();
  }
}
