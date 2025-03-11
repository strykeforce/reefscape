package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.AlgaeIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.funnel.FunnelIOFXS;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;

public class StopOpenLoopCommand extends SequentialCommandGroup {
  public StopOpenLoopCommand(
      RobotStateSubsystem robotStateSubsystem,
      CoralSubsystem coralSubsystem,
      FunnelIOFXS funnelIOFXS,
      AlgaeIOFX algaeIOFXS) {
    addCommands(
        new ResetCaseHealthCheckCommand(),
        new InstantCommand(() -> robotStateSubsystem.setState(RobotStates.IDLE, false)),
        new InstantCommand(() -> coralSubsystem.healthCheck()),
        new InstantCommand(() -> funnelIOFXS.setPct(0)),
        new InstantCommand(() -> algaeIOFXS.setPct(0)));
  }
}
