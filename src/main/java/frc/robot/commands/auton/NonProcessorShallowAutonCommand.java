package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.SetGyroOffsetCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.vision.SetVisionUpdatesCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;

public class NonProcessorShallowAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private PathHandler pathHandler;
  private DriveSubsystem driveSubsystem;
  private DriveAutonCommand startPath;

  public NonProcessorShallowAutonCommand(
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
      char startNode) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.pathHandler = pathHandler;
    this.driveSubsystem = driveSubsystem;

    startPath = new DriveAutonCommand(driveSubsystem, startPathName, true, true, false);

    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ZeroElevatorCommand(elevatorSubsystem),
                // new ResetOdometryCommand(
                // driveSubsystem, new Pose2d(7.1008875, 5.0756788, new Rotation2d(180))),
                new SequentialCommandGroup(
                    new ResetGyroCommand(driveSubsystem),
                    new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180))),
                new SetVisionUpdatesCommand(visionSubsystem, true)),
            startPath)) /*,
                        new frc.robot.commands.pathHandler.StartPathHandlerCommand(
                            pathHandler,
                            PathHandlerConstants.kShallowPathNames,
                            NodeNames,
                            NodeLevels,
                            startNode,
                            false)))*/;
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    pathHandler.reassignAlliance();
  }
}
