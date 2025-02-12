// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.algae.OpenLoopAlgaeCommand;
import frc.robot.commands.coral.EnableEjectBeamCommand;
import frc.robot.commands.coral.OpenLoopCoralCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.elevator.HoldElevatorCommand;
import frc.robot.commands.elevator.JogElevatorCommand;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.ScoreReefManualCommand;
import frc.robot.commands.robotState.SetScoringLevelCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.tagAlign.TagAlignCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.algae.AlgaeIOFX;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitIOFX;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelIOFXS;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final RobotStateSubsystem robotStateSubsystem;

  private final AlgaeIOFX algaeIO;
  private final AlgaeSubsystem algaeSubsystem;

  private final BattMonSubsystem battMonSubsystem;

  private final BiscuitIOFX biscuitIO;
  private final BiscuitSubsystem biscuitSubsystem;

  private final ClimbSubsystem climbSubsystem;

  private final CoralIO coralIO;
  private final CoralSubsystem coralSubsystem;

  private final Swerve swerve;
  private final DriveSubsystem driveSubsystem;

  private final ElevatorIO elevatorIO;
  private final ElevatorSubsystem elevatorSubsystem;

  private final FunnelIOFXS funnelIO;
  private final FunnelSubsystem funnelSubsystem;

  private final LEDIO ledIO;
  private final LEDSubsystem ledSubsystem;

  private final TagAlignSubsystem tagAlignSubsystem;

  private final VisionSubsystem visionSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);

  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {

    algaeIO = new AlgaeIOFX();
    algaeSubsystem = new AlgaeSubsystem(algaeIO);

    battMonSubsystem = new BattMonSubsystem();

    biscuitIO = new BiscuitIOFX();
    biscuitSubsystem = new BiscuitSubsystem(biscuitIO);

    climbSubsystem = new ClimbSubsystem();

    coralIO = new CoralIOFX();
    coralSubsystem = new CoralSubsystem(coralIO);

    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);

    elevatorIO = new ElevatorIOFX();
    elevatorSubsystem = new ElevatorSubsystem(elevatorIO);

    funnelIO = new FunnelIOFXS();
    funnelSubsystem = new FunnelSubsystem(funnelIO);

    ledIO = new LEDIO();
    ledSubsystem = new LEDSubsystem();

    visionSubsystem = new VisionSubsystem(driveSubsystem);

    tagAlignSubsystem = new TagAlignSubsystem(driveSubsystem, visionSubsystem);

    robotStateSubsystem =
        new RobotStateSubsystem(
            algaeSubsystem,
            battMonSubsystem,
            biscuitSubsystem,
            climbSubsystem,
            coralSubsystem,
            driveSubsystem,
            elevatorSubsystem,
            funnelSubsystem,
            ledSubsystem,
            tagAlignSubsystem,
            visionSubsystem);

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    configureTelemetry();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    coralSubsystem.registerWith(telemetryService);
    algaeSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    funnelSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(), () -> flysky.getStr(), () -> flysky.getYaw(), driveSubsystem));

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(new ScoreReefManualCommand(robotStateSubsystem, elevatorSubsystem, coralSubsystem));
  }

  private void configureOperatorBindings() {
    // Auto align testing!
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new TagAlignCommand(tagAlignSubsystem, driveSubsystem));

    // Set Levels
    new Trigger(() -> xboxController.getLeftTriggerAxis() > RobotConstants.kTriggerDeadband)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L1));
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L2));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L3));
    new Trigger(() -> xboxController.getRightTriggerAxis() > RobotConstants.kTriggerDeadband)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L4));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new StowCommand(
                robotStateSubsystem, elevatorSubsystem, coralSubsystem, biscuitSubsystem));

    // zero elevator, slated for removal
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));
  }

  private void configureTestOperatorBindings() {
    // Stop Coral
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, 0));

    // Intake Coral
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, 0.5))
        .onTrue(new EnableEjectBeamCommand(true, coralSubsystem));

    // Eject Coral
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, 1))
        .onTrue(new EnableEjectBeamCommand(false, coralSubsystem));

    // Move Elevator
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kTestingDeadband))
        .onTrue(
            new JogElevatorCommand(
                elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmountUp, Rotations)))
        .onFalse(new HoldElevatorCommand(elevatorSubsystem));
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kTestingDeadband))
        .onTrue(
            new JogElevatorCommand(
                elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmountDown, Rotations)))
        .onFalse(new HoldElevatorCommand(elevatorSubsystem));

    // Zero Elevator
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));

    // Algae Buttons
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new OpenLoopAlgaeCommand(algaeSubsystem, 0.5));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new OpenLoopAlgaeCommand(algaeSubsystem, -0.5));

    // Elevator setpoint testing
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(
            new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kFunnelSetpoint));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kStowSetpoint));
    (new Trigger(() -> xboxController.getPOV() == 0))
        .onTrue(
            new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kL1CoralSetpoint));
    (new Trigger(() -> xboxController.getPOV() == 90))
        .onTrue(
            new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kL2CoralSetpoint));
    (new Trigger(() -> xboxController.getPOV() == 180))
        .onTrue(
            new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kL3CoralSetpoint));
    (new Trigger(() -> xboxController.getPOV() == 270))
        .onTrue(
            new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.kL4CoralSetpoint));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void stopTagAlign() {
    tagAlignSubsystem.terminate();
  }
}
