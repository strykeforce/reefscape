package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.AutoScoreAlgaeCommand;
import frc.robot.commands.robotState.ForceBargeCommand;
import frc.robot.commands.robotState.ForceLowFloorAlgaeCommand;
import frc.robot.commands.robotState.SetAutoPlacingCommand;
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

public class IRIAutonCommand extends SequentialCommandGroup implements AutoCommandInterface {

  private DriveSubsystem driveSubsystem;
  private CoralSubsystem coralSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private VisionSubsystem visionSubsystem;

  private ArrayList<AutoCommandInterface> pathCommands = new ArrayList<>();

  public IRIAutonCommand(
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
      List<RobotStateSubsystem.ScoringLevel> algaeLevels,
      Pose2d startPose) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;

    // BooleanSupplier awayCondition =
    //     () -> DriverStation.getMatchTime() < AutonConstants.kBargeScoreMinTime;

    addCommands(
        new ParallelCommandGroup(
            new PrepOdomForAutoCommand(
                robotStateSubsystem, driveSubsystem, Rotation2d.fromDegrees(90.0), startPose),
            new ZeroElevatorCommand(elevatorSubsystem)),
        new ForceLowFloorAlgaeCommand(robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem),
        new SetAutoPlacingCommand(robotStateSubsystem, true),
        new ForceBargeCommand(
            robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem),
        new SetAutoPlacingCommand(robotStateSubsystem, false));

    for (int i = 0; i < grabPaths.size(); i++) {
      boolean last = i == grabPaths.size() - 1;
      var algaeDrive =
          new DriveAlgaeAutonServoCommand(
              driveSubsystem,
              tagAlignSubsystem,
              elevatorSubsystem,
              biscuitSubsystem,
              robotStateSubsystem,
              visionSubsystem,
              grabPaths.get(i),
              i == 0,
              true,
              i == 0,
              grabYOffsets.get(i),
              algaeLevels.get(i));
      var bargeDrive =
          last
              ? new DriveAutonCommand(driveSubsystem, bargePaths.get(i), true, false, false)
              : new DriveBargeAutonCommand(
                  driveSubsystem,
                  tagAlignSubsystem,
                  elevatorSubsystem,
                  biscuitSubsystem,
                  robotStateSubsystem,
                  visionSubsystem,
                  bargePaths.get(i),
                  true,
                  false);

      pathCommands.add(algaeDrive);
      pathCommands.add(bargeDrive);

      addCommands(algaeDrive, new WaitCommand(delays.get(i)), bargeDrive);

      if (!last) {
        addCommands(
            new AutoScoreAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));
      }
    }
    /*
    addCommands(
        new ConditionalCommand(
            new DriveAutonCommand(driveSubsystem, "bargeLeave", true, false, false),
            new SequentialCommandGroup(
                algaeDrive,
                new WaitCommand(delays.get(i))),
            awayCondition),
        new ConditionalCommand(
            new DriveAutonCommand(driveSubsystem, "bargeLeave", true, false, false),
            new SequentialCommandGroup(
                    bargeDrive,
                    new AutoScoreAlgaeCommand(
                        robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem))
                .until(awayCondition),
            awayCondition));
    }
    */

    // addCommands(new DriveAutonCommand(driveSubsystem, "bargeLeave", true, false, false));
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

    for (var command : pathCommands) {
      command.reassignAlliance();
    }
  }
}
