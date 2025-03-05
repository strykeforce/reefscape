package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.PrepOdomForAutoCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class DefaultAutonCommand extends SequentialCommandGroup
    implements AutoCommandInterface {

    private DriveSubsystem driveSubsystem;
    private DriveAutonCommand path;

    public DefaultAutonCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, String pathName, Pose2d startPose) {
        path = 
        new DriveAutonCommand(
            driveSubsystem,
            "defaultAuton",
            true,
            true, false);
        
        addCommands(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new PrepOdomForAutoCommand(driveSubsystem, Rotation2d.fromDegrees(180.0), startPose),
                new ZeroElevatorCommand(elevatorSubsystem)
            ),
            path
        ));
    }

    @Override
    public void reassignAlliance() {
    path.reassignAlliance();
    driveSubsystem.teleResetGyro();
    }
    
}
