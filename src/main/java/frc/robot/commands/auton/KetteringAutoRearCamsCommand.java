package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonRearCamCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.AutoForceBargeCommand;
import frc.robot.commands.robotState.MicAlgaeCommand;
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
import java.util.List;

public class KetteringAutoRearCamsCommand extends SequentialCommandGroup
    implements AutoCommandInterface {
  private DriveSubsystem driveSubsystem;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;

  private DriveAutonRearCamCommand bargeToAlgaePath;
  private DriveAlgaeAutonServoNoZeroCommand bargeToReefPath;
  private DriveAutonRearCamCommand reefToBargePath;
  private DriveAutonRearCamCommand bargeAwayPath;
  private DriveAutonRearCamCommand startPath;

  public KetteringAutoRearCamsCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      String startString,
      String bargeToAlgae,
      String bargeToReef,
      String reefToBarge,
      String bargeAway,
      List<RobotStateSubsystem.ScoringLevel> algaeLevels,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

    bargeToAlgaePath =
        new DriveAutonRearCamCommand(
            driveSubsystem, visionSubsystem, bargeToAlgae, true, false, false);
    bargeToReefPath =
        new DriveAlgaeAutonServoNoZeroCommand(
            driveSubsystem,
            tagAlignSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            robotStateSubsystem,
            visionSubsystem,
            bargeToReef,
            true,
            true,
            false,
            0,
            algaeLevels.get(0));
    reefToBargePath =
        new DriveAutonRearCamCommand(
            driveSubsystem, visionSubsystem, reefToBarge, true, false, false);
    bargeAwayPath =
        new DriveAutonRearCamCommand(
            driveSubsystem, visionSubsystem, bargeAway, true, false, false);
    startPath =
        new DriveAutonRearCamCommand(
            driveSubsystem, visionSubsystem, startString, true, true, false);

    addCommands(
        new ParallelCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(0.0), startPose),
            new ZeroElevatorCommand(elevatorSubsystem)),
        new ParallelCommandGroup(
            startPath,
            new MicAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem)),
        new AutoForceBargeCommand(
            robotStateSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            algaeSubsystem,
            visionSubsystem),
        new WaitForElevBelowBarge(elevatorSubsystem),
        new ParallelCommandGroup(
            bargeToAlgaePath,
            new MicAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem)),
        new AutoForceBargeCommand(
            robotStateSubsystem,
            elevatorSubsystem,
            biscuitSubsystem,
            algaeSubsystem,
            visionSubsystem),
        bargeToReefPath,
        reefToBargePath);
    // new AutoScoreAlgaeCommand(
    //     robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem),
    // bargeAwayPath);
  }

  @Override
  public void reassignAlliance() {
    driveSubsystem.teleResetGyro();
    coralSubsystem.setAutoPreload();
    robotStateSubsystem.setIsAutoPlacing(true);
    robotStateSubsystem.setScoringLevel(ScoringLevel.L4);
    robotStateSubsystem.setGetAlgaeOnCycle(true);
    robotStateSubsystem.setScoreSide(ScoreSide.LEFT);
    visionSubsystem.setVisionUpdating(true);

    bargeToAlgaePath.reassignAlliance();
    bargeToReefPath.reassignAlliance();
    reefToBargePath.reassignAlliance();
    bargeAwayPath.reassignAlliance();
    startPath.reassignAlliance();
  }
}
