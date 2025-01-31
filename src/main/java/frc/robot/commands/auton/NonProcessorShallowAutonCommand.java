package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathHandler.KillPathHandlerCommand;
import frc.robot.commands.PathHandler.StartPathHandlerCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class NonProcessorShallowAutonCommand extends SequentialCommandGroup {

    private PathHandler pathHandler;
    private DriveSubsystem driveSubsystem;
    private DriveAutonCommand startPath;

    public NonProcessorShallowAutonCommand (
        DriveSubsystem driveSubsystem,
        RobotStateSubsystem robotStateSubsystem,
        AlgaeSubsystem algaeSubsystem,
        BiscuitSubsystem biscuitSubsystem,
        CoralSubsystem coralSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        PathHandler pathHandler,
        String startPathName,
        String[][] pathNames,
        List<Character> NodeNames,
        List<Integer> NodeLevels,
        boolean mirrorToProcessor
    ) {
        // addRequirements(driveSubsystem, algaeSubsystem, biscuitSubsystem, coralSubsystem, elevatorSubsystem);
        this.pathHandler = pathHandler;
        this.driveSubsystem = driveSubsystem;
        this.pathHandler.setPathNames(pathNames);
        this.pathHandler.setStartNode(NodeNames.remove(0));
        this.pathHandler.setNodeNames(NodeNames);
        this.pathHandler.setNodeLevels(NodeLevels);
        this.pathHandler.setMirrorToProcessor(mirrorToProcessor);
        
        startPath = new DriveAutonCommand(driveSubsystem, startPathName, false, true, mirrorToProcessor);

        addCommands(
            new SequentialCommandGroup(
                // TODO fill in all zeroing and such
                startPath,
                new StartPathHandlerCommand(pathHandler),
                new KillPathHandlerCommand(pathHandler)
            )
        );
    }

    public void reassignAlliance() {
        startPath.reassignAlliance();
    }
    
}
