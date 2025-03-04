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
import java.util.function.BooleanSupplier;

public class ProcessorShallowAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private PathHandler pathHandler;
  private DriveSubsystem driveSubsystem;
  private DriveAutonServoCommand startPath;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;

  public ProcessorShallowAutonCommand(
      DriveSubsystem driveSubsystem,
      PathHandler pathHandler,
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      BooleanSupplier button,
      String startPathName,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      char startNode,
      boolean startScoreLeft,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.pathHandler = pathHandler;
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

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
            new PrepOdomForAutoCommand(driveSubsystem, Rotation2d.fromDegrees(180), startPose),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            startPath,
            new PlaceCoralAutonCommand(robotStateSubsystem, coralSubsystem),

            // new ParallelCommandGroup(
            //     new ZeroElevatorCommand(elevatorSubsystem),
            // new ResetOdometryCommand(
            // driveSubsystem, new Pose2d(7.1008875, 5.0756788, new Rotation2d(180))),
            // new SequentialCommandGroup(
            //     new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            //     new ResetOdometryCommand(driveSubsystem, AutonConstants.kNonProcessorShallow))),
            // new ParallelCommandGroup(
            // //     new WaitCommand(0.03), new SetVisionUpdatesCommand(visionSubsystem, true)),
            // startPath,
            // new WaitForButtonPressCommand(button),
            new StartPathHandlerCommand(
                pathHandler,
                PathHandlerConstants.kProcessorShallowPathNames,
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
    robotStateSubsystem.setIsAuto(true);
    robotStateSubsystem.setScoreSide(ScoreSide.RIGHT);
    visionSubsystem.setVisionUpdating(true);
    pathHandler.setPathNames(PathHandlerConstants.kProcessorShallowPathNames);
    // pathHandler.reassignAlliance();
  }
}
