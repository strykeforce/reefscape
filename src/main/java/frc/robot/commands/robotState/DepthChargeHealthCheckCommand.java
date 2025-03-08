package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.AlgaeIOFX;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.biscuit.BiscuitIOFXS;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveFXS;
import frc.robot.subsystems.elevator.ElevatorIOFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelIOFXS;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import java.util.List;
import org.strykeforce.healthcheck.IOHealthCheckCommand;

public class DepthChargeHealthCheckCommand extends SequentialCommandGroup {
  private SwerveFXS swerve;
  private DriveSubsystem driveSubsystem;
  private FunnelIOFXS funnelIOFXS;
  private FunnelSubsystem funnelSubsystem;
  private CoralIOFX coralIOFXS;
  private CoralSubsystem coralSubsystem;
  private AlgaeIOFX algaeIOFXS;
  private AlgaeSubsystem algaeSubsystem;
  private ElevatorIOFX elevatorIOFX;
  private ElevatorSubsystem elevatorSubsystem;
  private BiscuitIOFXS biscuitIOFXS;
  private BiscuitSubsystem biscuitSubsystem;

  public DepthChargeHealthCheckCommand(
      RobotStateSubsystem robotStateSubsystem,
      SwerveFXS swerve,
      DriveSubsystem driveSubsystem,
      FunnelIOFXS funnelIOFXS,
      FunnelSubsystem funnelSubsystem,
      CoralIOFX coralIOFXS,
      CoralSubsystem coralSubsystem,
      AlgaeIOFX algaeIOFXS,
      AlgaeSubsystem algaeSubsystem,
      ElevatorIOFX elevatorIOFX,
      ElevatorSubsystem elevatorSubsystem,
      BiscuitIOFXS biscuitIOFXS,
      BiscuitSubsystem biscuitSubsystem) {

    this.swerve = swerve;
    this.driveSubsystem = driveSubsystem;
    this.funnelIOFXS = funnelIOFXS;
    this.funnelSubsystem = funnelSubsystem;
    this.coralIOFXS = coralIOFXS;
    this.coralSubsystem = coralSubsystem;
    this.algaeIOFXS = algaeIOFXS;
    this.algaeSubsystem = algaeSubsystem;
    this.elevatorIOFX = elevatorIOFX;
    this.elevatorSubsystem = elevatorSubsystem;
    this.biscuitIOFXS = biscuitIOFXS;
    this.biscuitSubsystem = biscuitSubsystem;

    addCommands(
        new ResetCaseHealthCheckCommand(),
        new InstantCommand(() -> robotStateSubsystem.setState(RobotStates.IDLE, false)),
        new InstantCommand(() -> coralSubsystem.healthCheck()),
        new InstantCommand(() -> funnelIOFXS.setPct(0)),
        new InstantCommand(() -> algaeIOFXS.setPct(0)),
        new IOHealthCheckCommand(
            List.of(
                driveSubsystem,
                funnelSubsystem,
                coralSubsystem,
                algaeSubsystem,
                elevatorSubsystem,
                biscuitSubsystem),
            swerve,
            funnelIOFXS,
            coralIOFXS,
            algaeIOFXS,
            elevatorIOFX,
            biscuitIOFXS),
        new LockWheelsCommand(driveSubsystem),
        new StowCommand(
            robotStateSubsystem,
            elevatorSubsystem,
            coralSubsystem,
            biscuitSubsystem,
            algaeSubsystem));
  }
}
