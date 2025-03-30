package frc.robot.commands.pathHandler;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import java.util.List;

public class SetPathHandlerCommand extends InstantCommand {
  private PathHandler pathHandler;
  private String[][] pathNames;
  private List<Character> NodeNames;
  private List<ScoringLevel> NodeLevels;
  private Character startNode;
  private boolean mirrorToProcessor;

  public SetPathHandlerCommand(
      PathHandler pathHandler,
      String[][] pathNames,
      List<Character> NodeNames,
      List<ScoringLevel> NodeLevels,
      Character startNode,
      boolean mirrorToProcessor) {
    this.pathHandler = pathHandler;
    this.pathNames = pathNames;
    this.NodeNames = NodeNames;
    this.NodeLevels = NodeLevels;
    this.startNode = startNode;
    this.mirrorToProcessor = mirrorToProcessor;
  }

  @Override
  public void initialize() {
    pathHandler.setPathNames(pathNames);
    pathHandler.setNodeNames(NodeNames);
    pathHandler.setNodeLevels(NodeLevels);
    pathHandler.setStartNode(startNode);
    pathHandler.setMirrorToProcessor(mirrorToProcessor);
  }
}
