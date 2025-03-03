package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAutonServoCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.drive.SetGyroOffsetCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.pathHandler.StartPathHandlerCommand;
import frc.robot.commands.vision.SetVisionUpdatesCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.PathHandlerConstants;
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
import java.util.function.BooleanSupplier;

public class NonProcessorShallowAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

  private PathHandler pathHandler;
  private DriveSubsystem driveSubsystem;
  private DriveAutonServoCommand startPath;

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
      BooleanSupplier button,
      String startPathName,
      List<Character> NodeNames,
      List<Integer> NodeLevels,
      char startNode) {
    addRequirements(
        driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
    this.pathHandler = pathHandler;
    this.driveSubsystem = driveSubsystem;

    startPath =
        new DriveAutonServoCommand(
            driveSubsystem, tagAlignSubsystem, startPathName, true, true, false, false);

    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ZeroElevatorCommand(elevatorSubsystem),
                // new ResetOdometryCommand(
                // driveSubsystem, new Pose2d(7.1008875, 5.0756788, new Rotation2d(180))),
                new SequentialCommandGroup(
                    new SetGyroOffsetCommand(driveSubsystem, Rotation2d.fromDegrees(180)),
                    new ResetOdometryCommand(driveSubsystem, AutonConstants.kNonProcessorShallow))),
            new ParallelCommandGroup(
                new WaitCommand(0.03), new SetVisionUpdatesCommand(visionSubsystem, true)),
            startPath,
            new WaitForButtonPressCommand(button),
            new StartPathHandlerCommand(
                pathHandler,
                PathHandlerConstants.kShallowPathNames,
                NodeNames,
                NodeLevels,
                startNode,
                false)));
  }

  @Override
  public void reassignAlliance() {
    startPath.reassignAlliance();
    driveSubsystem.teleResetGyro();
    // pathHandler.reassignAlliance();
  }
}
