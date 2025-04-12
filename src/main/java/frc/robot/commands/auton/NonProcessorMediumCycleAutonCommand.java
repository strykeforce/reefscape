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
import java.util.ArrayList;
import java.util.List;

public class NonProcessorMediumCycleAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private PathHandler pathHandler;
  private DriveSubsystem driveSubsystem;
  private DriveAutonServoCommand startPath;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;
  private List<Character> NodeNames;
  private List<ScoringLevel> NodeLevels;
  private List<Double> lightDistances = new ArrayList<>();
  private List<Double> postOffsets = new ArrayList<>();
  private char startNode;
  private boolean lastAlgae;

  public NonProcessorMediumCycleAutonCommand(
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
      List<ScoringLevel> NodeLevels,
      List<Double> lightDistances,
      List<Double> postOffsets,
      char startNode,
      boolean startScoreLeft,
      boolean lastAlgae,
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
    this.lastAlgae = lastAlgae;

    this.lightDistances = lightDistances;
    this.postOffsets = postOffsets;

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
            startScoreLeft,
            postOffsets.get(0));
    postOffsets.remove(0);

    addCommands(
        new SequentialCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(180.0), startPose),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            startPath,
            new PlaceCoralAutonCommand(robotStateSubsystem, coralSubsystem),
            new StartPathHandlerCommand(
                pathHandler,
                PathHandlerConstants.kShallowPathNames,
                NodeNames,
                NodeLevels,
                startNode,
                false)),
        // TODO Make function to detect and score coral before we drive the last path
        // TODO Make sure it works
        // TODO Make one like this for the barge
        new DriveAutonServoCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            "KTofetchSlow.traj",
            true,
            false,
            false,
            true,
            0.0));
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
    coralSubsystem.setAutoPreload();
    robotStateSubsystem.setIsAutoPlacing(false);
    robotStateSubsystem.setScoringLevel(ScoringLevel.L4);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    // robotStateSubsystem.setIsAuto(true);
    robotStateSubsystem.setScoreSide(ScoreSide.RIGHT);
    visionSubsystem.setVisionUpdating(true);
    pathHandler.setPathNames(PathHandlerConstants.kProcessorSlowShallowPathNames);
    pathHandler.setNodeNames(NodeNames);
    pathHandler.setNodeLevels(NodeLevels);
    pathHandler.setStartNode(startNode);
    pathHandler.setGetAlgaeLast(lastAlgae);
    pathHandler.setLightDistances(lightDistances);
    pathHandler.setPostOffsets(postOffsets);
    // pathHandler.reassignAlliance();
  }
}
