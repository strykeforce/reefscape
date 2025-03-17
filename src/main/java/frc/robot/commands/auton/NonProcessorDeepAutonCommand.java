package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;

public class NonProcessorDeepAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private PathHandler pathHandler;
  private DriveSubsystem driveSubsystem;
  private DriveAutonServoCommand startPath;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;
  private List<Character> NodeNames;
  private List<Integer> NodeLevels;
  private char startNode;

  public NonProcessorDeepAutonCommand(
      DriveSubsystem driveSubsystem,
      PathHandler pathHandler,
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      String startPathName,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      char startNode,
      Boolean startScoreLeft,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.pathHandler = pathHandler;
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

    this.NodeNames = NodeNames;
    this.NodeLevels = NodeLevels;
    this.startNode = startNode;

    startPath =
        new DriveAutonServoCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            startPathName,
            true,
            true,
            false,
            startScoreLeft);

    addCommands(
        new SequentialCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(180.0), startPose),
            startPath,
            new PlaceCoralAutonCommand(robotStateSubsystem, coralSubsystem),
            new StartPathHandlerCommand(
                pathHandler,
                PathHandlerConstants.kDeepPathNames,
                NodeNames,
                NodeLevels,
                startNode,
                false)));
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
    coralSubsystem.setAutoPreload();
    robotStateSubsystem.setIsAutoPlacing(false);
    robotStateSubsystem.setScoringLevel(ScoringLevel.L4);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    robotStateSubsystem.setScoreSide(ScoreSide.RIGHT);
    visionSubsystem.setVisionUpdating(true);
    pathHandler.setPathNames(PathHandlerConstants.kDeepPathNames);
    pathHandler.setNodeNames(NodeNames);
    pathHandler.setNodeLevels(NodeLevels);
    pathHandler.setStartNode(startNode);
  }
}
