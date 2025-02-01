// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.coral.EnableEjectBeamCommand;
import frc.robot.commands.coral.OpenLoopCoralCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private Swerve swerve;
  private DriveSubsystem driveSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);

  private final FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private final CoralSubsystem coralSubsystem;
  private final CoralIO coralIO;

  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);

    coralIO = new CoralIOFX();
    coralSubsystem = new CoralSubsystem(coralIO);

    configureTelemetry();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureTelemetry() {
    telemetryService.register(driveSubsystem);
    telemetryService.register(coralSubsystem);
    telemetryService.start();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(), () -> flysky.getStr(), () -> flysky.getYaw(), driveSubsystem));
  }

  private void configureOperatorBindings() {
    // Stop Coral
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, 0));

    // Intake Coral
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, -0.5))
        .onTrue(new EnableEjectBeamCommand(false, coralSubsystem));

    // Eject Coral
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, -0.5))
        .onTrue(new EnableEjectBeamCommand(true, coralSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
