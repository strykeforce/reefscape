// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.elevator.JogElevatorCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotContainer {

  private ElevatorIO elevatorIO = new ElevatorIOFX();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorIO);

  private final XboxController xboxController = new XboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kJoystickDeadband))
        .onTrue(new JogElevatorCommand(elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmount, Rotations)));
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kJoystickDeadband))
        .onTrue(new JogElevatorCommand(elevatorSubsystem, Angle.ofBaseUnits(-ElevatorConstants.kJogAmount, Rotations)));
    new JoystickButton(xboxController, XboxController.Button.kA.value)
      .onTrue(new ZeroElevatorCommand(elevatorSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
