package frc.robot.commands.pathhHandler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pathHandler.PathHandler;
import java.util.List;

public class StartPathHandlerCommand extends Command {
  private PathHandler pathHandler;
  private String[][] pathNames;
  private List<Character> NodeNames;
  private List<Integer> NodeLevels;
  private Character startNode;
  private boolean mirrorToProcessor;

  public StartPathHandlerCommand(
      PathHandler pathHandler,
      String[][] pathNames,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      Character startNode,
      boolean mirrorToProcessor) {
    this.pathHandler = pathHandler;
    this.pathNames = pathNames;
    this.NodeNames = NodeNames;
    this.NodeLevels = NodeLevels;
    this.startNode = startNode;
    this.mirrorToProcessor = mirrorToProcessor;
  }

  public StartPathHandlerCommand(PathHandler pathHandler) {
    this.pathHandler = pathHandler;
  }

  @Override
  public void initialize() {
    if (pathNames != null) {
      pathHandler.setPathNames(pathNames);
      pathHandler.setNodeNames(NodeNames);
      pathHandler.setNodeLevels(NodeLevels);
      pathHandler.setStartNode(startNode);
      pathHandler.setMirrorToProcessor(mirrorToProcessor);
    }
    pathHandler.startPathHandler();
  }

  @Override
  public boolean isFinished() {
    return pathHandler.isFinished();
  }
}
