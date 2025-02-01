// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.coral.OpenLoopCoralCommand;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;

public class RobotContainer {
  private final CoralSubsystem coralSubsystem;
  private final CoralIO coralIO;

  private final XboxController xboxController = new XboxController(1);

  public RobotContainer() {
    coralIO = new CoralIOFX();
    coralSubsystem = new CoralSubsystem(coralIO);

    configureBindings();
    configureOperatorBindings();
  }

  private void configureBindings() {}

  private void configureOperatorBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, 0.5));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new OpenLoopCoralCommand(coralSubsystem, -0.5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
