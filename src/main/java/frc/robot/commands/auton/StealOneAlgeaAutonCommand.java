package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;

public class StealOneAlgeaAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private DriveSubsystem driveSubsystem;
  private DriveAlgaeAutonServoCommand firstPath;
  private DriveBargeAutonCommand secondPath;
  private DriveAlgaeAutonServoCommand thirdPath;
  private DriveBargeAutonCommand fourthPath;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;
  private List<Double> postOffsets = new ArrayList<>();

  public StealOneAlgeaAutonCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      String firstPathName,
      String secondPathName,
      String thirdPathName,
      String fourthPathName,
      List<Double> postOffsets,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.postOffsets = postOffsets;

    firstPath =
        new DriveAlgaeAutonServoCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            visionSubsystem,
            firstPathName,
            true,
            true,
            true,
            postOffsets.get(0));

    secondPath =
        new DriveBargeAutonCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            visionSubsystem,
            secondPathName,
            true,
            false);

    thirdPath =
        new DriveAlgaeAutonServoCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            visionSubsystem,
            thirdPathName,
            false,
            true,
            false,
            postOffsets.get(1));

    fourthPath =
        new DriveBargeAutonCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            visionSubsystem,
            fourthPathName,
            true,
            false);

    addCommands(
        new SequentialCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(180.0), startPose),
            // new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
            firstPath,
            new PlaceCoralAutonCommand(robotStateSubsystem, coralSubsystem),
            secondPath,
            // score Barge
            new WaitForElevBelowBarge(elevatorSubsystem),
            thirdPath,
            fourthPath));
  }

  @Override
  public void reassignAlliance() {
    firstPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
    coralSubsystem.setAutoPreload();
    robotStateSubsystem.setIsAutoPlacing(false);
    robotStateSubsystem.setScoringLevel(ScoringLevel.L4);
    robotStateSubsystem.setGetAlgaeOnCycle(false);
    // robotStateSubsystem.setIsAuto(true);
    robotStateSubsystem.setScoreSide(ScoreSide.RIGHT);
    visionSubsystem.setVisionUpdating(true);
    // pathHandler.reassignAlliance();
  }
}
