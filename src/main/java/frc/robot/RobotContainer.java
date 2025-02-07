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
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.algae.AlgaeIOFX;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final AlgaeIOFX algaeIO;
  private final AlgaeSubsystem algaeSubsystem;

  private final CoralIO coralIO;
  private final CoralSubsystem coralSubsystem;

  private final Swerve swerve;
  private final DriveSubsystem driveSubsystem;

  private final ElevatorIO elevatorIO;
  private final ElevatorSubsystem elevatorSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);

  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {
    algaeIO = new AlgaeIOFX();
    algaeSubsystem = new AlgaeSubsystem(algaeIO);

    coralIO = new CoralIOFX();
    coralSubsystem = new CoralSubsystem(coralIO);

    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);

    elevatorIO = new ElevatorIOFX();
    elevatorSubsystem = new ElevatorSubsystem(elevatorIO);

    configureTelemetry();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureTelemetry() {
    telemetryService.register(driveSubsystem);
    telemetryService.register(coralSubsystem);
    telemetryService.register(algaeSubsystem);
    telemetryService.register(elevatorSubsystem);
    elevatorIO.registerWith(telemetryService);
    telemetryService.start();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(), () -> flysky.getStr(), () -> flysky.getYaw(), driveSubsystem));

    // Reset Gyro Command
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));
  }

  private void configureOperatorBindings() {
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
}
