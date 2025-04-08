package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.robotState.ScoreAlgaeCommand;
import frc.robot.constants.AutonConstants;
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

public class MiddleBargeAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private DriveSubsystem driveSubsystem;
  private DriveAutonServoCommand startPath;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;

  public MiddleBargeAutonCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      List<String> grabPaths,
      List<Double> grabYOffsets,
      List<Double> delays,
      List<String> bargePaths,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

    addCommands(
        new PrepOdomForAutoCommand(
            robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(180.0), startPose));

    for (int i = 0; i < grabPaths.size(); i++) {
      addCommands(
          new DriveAlgaeAutonServoCommand(
              driveSubsystem,
              tagAlignSubsystem,
              elevatorSubsystem,
              biscuitSubsystem,
              robotStateSubsystem,
              grabPaths.get(i),
              i == 0,
              false,
              i == 0,
              grabYOffsets.get(i)),
          new WaitCommand(delays.get(i)),
          new ConditionalCommand(
              new SequentialCommandGroup(
                  new DriveBargeAutonCommand(
                      driveSubsystem,
                      tagAlignSubsystem,
                      elevatorSubsystem,
                      biscuitSubsystem,
                      robotStateSubsystem,
                      bargePaths.get(i),
                      i == grabPaths.size() - 1,
                      false),
                  new ScoreAlgaeCommand(
                      robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem)),
              new WaitCommand(15),
              () -> DriverStation.getMatchTime() > AutonConstants.kBargeScoreMinTime));
    }
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
    coralSubsystem.setAutoPreload();
    robotStateSubsystem.setIsAutoPlacing(false);
    robotStateSubsystem.setScoringLevel(ScoringLevel.L4);
    robotStateSubsystem.setGetAlgaeOnCycle(true);
    robotStateSubsystem.setScoreSide(ScoreSide.LEFT);
    visionSubsystem.setVisionUpdating(true);
  }
}
