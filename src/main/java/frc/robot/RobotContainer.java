// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.algae.IntakeAlgaeCommand;
import frc.robot.commands.algae.OpenLoopAlgaeCommand;
import frc.robot.commands.algae.ProcessorAlgaeCommand;
import frc.robot.commands.auton.NonProcessorShallowAutonCommand;
import frc.robot.commands.auton.ProcessorShallowAutonCommand;
import frc.robot.commands.auton.ToggleVirtualSwitchCommand;
import frc.robot.commands.biscuit.HoldBiscuitCommand;
import frc.robot.commands.biscuit.JogBiscuitCommand;
import frc.robot.commands.biscuit.ZeroBiscuitCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.ClimbPrepCommand;
import frc.robot.commands.coral.EnableEjectBeamCommand;
import frc.robot.commands.coral.OpenLoopCoralCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ResetGyroCommand;
import frc.robot.commands.drive.SpinUpWheelsCommand;
import frc.robot.commands.elevator.HoldElevatorCommand;
import frc.robot.commands.elevator.JogElevatorCommand;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.AutoReefCycleCommand;
import frc.robot.commands.robotState.FloorAlgaeCommand;
import frc.robot.commands.robotState.ForceProcessorCommand;
import frc.robot.commands.robotState.HPAlgaeCommand;
import frc.robot.commands.robotState.InterruptAutoCommand;
import frc.robot.commands.robotState.ReefCycleCommand;
import frc.robot.commands.robotState.ScoreAlgaeCommand;
import frc.robot.commands.robotState.SetScoreSideCommand;
import frc.robot.commands.robotState.SetScoreSideRightCommand;
import frc.robot.commands.robotState.SetScoringLevelCommand;
import frc.robot.commands.robotState.StopAllAxisCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.ToggleAlgaeHeightCommand;
import frc.robot.commands.robotState.ToggleAutoPlacingCommand;
import frc.robot.commands.robotState.ToggleGetAlgaeCommand;
import frc.robot.commands.robotState.lockwheelscommand;
import frc.robot.commands.robotState.setScoreSideLeftCommand;
import frc.robot.commands.vision.SetVisionUpdatesCommand;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.algae.AlgaeIOFX;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.auto.AutoSwitch;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitIOFXS;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOServoFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOFX;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveFXS;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.funnel.FunnelIOFXS;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Map;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final RobotConstants robotConstants;
  private final RobotStateSubsystem robotStateSubsystem;

  private final AlgaeIOFX algaeIO;
  private final AlgaeSubsystem algaeSubsystem;

  private final BattMonSubsystem battMonSubsystem;

  private final BiscuitIOFXS biscuitIO;
  private final BiscuitSubsystem biscuitSubsystem;

  private final ClimbIO climbIO;
  private final ClimbSubsystem climbSubsystem;

  private final CoralIO coralIO;
  private final CoralSubsystem coralSubsystem;

  private Swerve protoSwerve;
  private SwerveFXS swerve;
  private final DriveSubsystem driveSubsystem;

  private final ElevatorIO elevatorIO;
  private final ElevatorSubsystem elevatorSubsystem;

  private final FunnelIOFXS funnelIO;
  private final FunnelSubsystem funnelSubsystem;

  private final LEDIO ledIO;
  private final LEDSubsystem ledSubsystem;

  private final TagAlignSubsystem tagAlignSubsystem;

  private final VisionSubsystem visionSubsystem;

  private final PathHandler pathHandler;

  private final AutoSwitch autoSwitch;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);

  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private NonProcessorShallowAutonCommand nonProcessorShallowAutonCommand;
  private ProcessorShallowAutonCommand processorShallowAutonCommand;

  private Alliance alliance = Alliance.Blue;
  private SuppliedValueWidget<Boolean> allianceColor;

  public RobotContainer() {
    robotConstants = new RobotConstants();

    algaeIO = new AlgaeIOFX();
    algaeSubsystem = new AlgaeSubsystem(algaeIO);

    battMonSubsystem = new BattMonSubsystem();

    biscuitIO = new BiscuitIOFXS();
    biscuitSubsystem = new BiscuitSubsystem(biscuitIO);

    climbIO = new ClimbIOServoFX();
    climbSubsystem = new ClimbSubsystem(climbIO);

    coralIO = new CoralIOFX();
    coralSubsystem = new CoralSubsystem(coralIO);

    if (RobotConstants.isComp) {
      swerve = new SwerveFXS();
      driveSubsystem = new DriveSubsystem(swerve);
    } else {
      protoSwerve = new Swerve();
      driveSubsystem = new DriveSubsystem(protoSwerve);
    }

    elevatorIO = new ElevatorIOFX();
    elevatorSubsystem = new ElevatorSubsystem(elevatorIO);

    funnelIO = new FunnelIOFXS();
    funnelSubsystem = new FunnelSubsystem(funnelIO);

    ledIO = new LEDIO();
    ledSubsystem = new LEDSubsystem(ledIO);

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

    pathHandler = new PathHandler(driveSubsystem, tagAlignSubsystem, robotStateSubsystem);

    autoSwitch =
        new AutoSwitch(
            robotStateSubsystem,
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
            visionSubsystem,
            pathHandler);

    configureTelemetry();
    configureDriverBindings();
    configureOperatorBindings();
    configureMatchDashboard();
    configTestDash();
    configurePitDashboard();
    robotStateSubsystem.setAllianceColor(Alliance.Blue);
  }

  public boolean hasBiscuitZeroed() {
    return biscuitSubsystem.hasZeroed();
  }

  public void zeroBiscuit() {
    biscuitSubsystem.zero();
  }

  public boolean hasSwerveZeroed() {
    return driveSubsystem.hasZeroed();
  }

  public void zeroSwerve() {
    driveSubsystem.zeroModules();
  }

  public boolean hasElevatorZeroed() {
    return elevatorSubsystem.getState() == ElevatorStates.ZEROED;
  }

  public void zeroElevator() {
    robotStateSubsystem.startupSequence();
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    coralSubsystem.registerWith(telemetryService);
    algaeSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    funnelSubsystem.registerWith(telemetryService);
    biscuitSubsystem.registerWith(telemetryService);
    ledSubsystem.registerWith(telemetryService);
    climbSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  public Alliance getAllianceColor() {
    return alliance;
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "blue"));
    robotStateSubsystem.setAllianceColor(alliance);

    autoSwitch.getAutoCommand().reassignAlliance();

    if (robotStateSubsystem.getAllianceColor() == Alliance.Red)
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    else driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(),
            () -> flysky.getStr(),
            () -> flysky.getYaw(),
            driveSubsystem,
            robotStateSubsystem));

    // Reset Gyro Command, stow, interrupt Auton, and zero elev
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new ResetGyroCommand(driveSubsystem));

    // Force Processor
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(
            new ForceProcessorCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem))
        .onFalse(
            new ForceProcessorCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));

    // Interupt
    new JoystickButton(driveJoystick, Button.SWG_UP.id)
        .onTrue(new InterruptAutoCommand(robotStateSubsystem, coralSubsystem))
        .onFalse(new InterruptAutoCommand(robotStateSubsystem, coralSubsystem));
    new JoystickButton(driveJoystick, Button.SWG_DWN.id)
        .onTrue(new InterruptAutoCommand(robotStateSubsystem, coralSubsystem))
        .onFalse(new InterruptAutoCommand(robotStateSubsystem, coralSubsystem));

    new JoystickButton(driveJoystick, Button.SWB_UP.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem))
        .onFalse(new ZeroElevatorCommand(elevatorSubsystem));
    new JoystickButton(driveJoystick, Button.SWB_DWN.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem))
        .onFalse(new ZeroElevatorCommand(elevatorSubsystem));

    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(
            new ConditionalCommand(
                new AutoReefCycleCommand(
                    robotStateSubsystem,
                    elevatorSubsystem,
                    coralSubsystem,
                    driveSubsystem,
                    tagAlignSubsystem,
                    biscuitSubsystem,
                    algaeSubsystem),
                new ReefCycleCommand(
                    robotStateSubsystem,
                    elevatorSubsystem,
                    coralSubsystem,
                    biscuitSubsystem,
                    algaeSubsystem),
                () -> robotStateSubsystem.getIsAutoPlacing()));

    new JoystickButton(driveJoystick, Button.M_SWE.id)
        .onTrue(
            new ScoreAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));
    new JoystickButton(driveJoystick, Button.SWF_UP.id)
        .onTrue(
            new FloorAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem))
        .onFalse(
            new FloorAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));
    new JoystickButton(driveJoystick, Button.SWF_DWN.id)
        .onTrue(
            new FloorAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem))
        .onFalse(
            new FloorAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));

    // climb
    new JoystickButton(driveJoystick, Button.SWA.id)
        .onTrue(new ClimbCommand(robotStateSubsystem, climbSubsystem));
  }

  private void configureOperatorBindings() {
    // Set Levels
    new Trigger(() -> xboxController.getLeftTriggerAxis() > RobotConstants.kTriggerDeadband)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L1));
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L2));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L3));
    new Trigger(() -> xboxController.getRightTriggerAxis() > RobotConstants.kTriggerDeadband)
        .onTrue(new SetScoringLevelCommand(robotStateSubsystem, ScoringLevel.L4));

    // Set scoring side
    new JoystickButton(xboxController, XboxController.Button.kLeftStick.value)
        .onTrue(new SetScoreSideCommand(robotStateSubsystem, ScoreSide.LEFT));
    new JoystickButton(xboxController, XboxController.Button.kRightStick.value)
        .onTrue(new SetScoreSideCommand(robotStateSubsystem, ScoreSide.RIGHT));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new StowCommand(
                robotStateSubsystem,
                elevatorSubsystem,
                coralSubsystem,
                biscuitSubsystem,
                algaeSubsystem));

    // Algae
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new ToggleAlgaeHeightCommand(robotStateSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ToggleGetAlgaeCommand(robotStateSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(
            new HPAlgaeCommand(
                robotStateSubsystem, elevatorSubsystem, biscuitSubsystem, algaeSubsystem));

    // Scoring
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new ToggleAutoPlacingCommand(robotStateSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kRightStick.value)
        .onTrue(new SetScoreSideRightCommand(robotStateSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kLeftStick.value)
        .onTrue(new setScoreSideLeftCommand(robotStateSubsystem));

    // Prep Climb
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(
            new ClimbPrepCommand(
                robotStateSubsystem, climbSubsystem, elevatorSubsystem, biscuitSubsystem));
  }

  private void configureTestOperatorBindings() {
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new IntakeAlgaeCommand(algaeSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new ProcessorAlgaeCommand(algaeSubsystem));

    // Move biscuit
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kTestingDeadband))
        .onTrue(
            new JogBiscuitCommand(
                biscuitSubsystem, Angle.ofBaseUnits(BiscuitConstants.kJogAmountUp, Rotations)))
        .onFalse(new HoldBiscuitCommand(biscuitSubsystem));
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kTestingDeadband))
        .onTrue(
            new JogBiscuitCommand(
                biscuitSubsystem, Angle.ofBaseUnits(BiscuitConstants.kJogAmountDown, Rotations)))
        .onFalse(new HoldBiscuitCommand(biscuitSubsystem));

    // Move elevator
    new Trigger((() -> xboxController.getLeftY() < -RobotConstants.kTestingDeadband))
        .onTrue(
            new JogElevatorCommand(
                elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmountUp, Rotations)))
        .onFalse(new HoldElevatorCommand(elevatorSubsystem));
    new Trigger((() -> xboxController.getLeftY() > RobotConstants.kTestingDeadband))
        .onTrue(
            new JogElevatorCommand(
                elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmountDown, Rotations)))
        .onFalse(new HoldElevatorCommand(elevatorSubsystem));

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

    // Move biscuit
    new Trigger((() -> xboxController.getRightY() < -RobotConstants.kTestingDeadband))
        .onTrue(
            new JogBiscuitCommand(
                biscuitSubsystem, Angle.ofBaseUnits(BiscuitConstants.kJogAmountUp, Rotations)))
        .onFalse(new HoldBiscuitCommand(biscuitSubsystem));
    new Trigger((() -> xboxController.getRightY() > RobotConstants.kTestingDeadband))
        .onTrue(
            new JogBiscuitCommand(
                biscuitSubsystem, Angle.ofBaseUnits(BiscuitConstants.kJogAmountDown, Rotations)))
        .onFalse(new HoldBiscuitCommand(biscuitSubsystem));

    // Move elevator
    new Trigger((() -> xboxController.getLeftY() < -RobotConstants.kTestingDeadband))
        .onTrue(
            new JogElevatorCommand(
                elevatorSubsystem, Angle.ofBaseUnits(ElevatorConstants.kJogAmountUp, Rotations)))
        .onFalse(new HoldElevatorCommand(elevatorSubsystem));
    new Trigger((() -> xboxController.getLeftY() > RobotConstants.kTestingDeadband))
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
            new SetElevatorPositionCommand(
                elevatorSubsystem, RobotConstants.kElevatorFunnelSetpoint));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new SetElevatorPositionCommand(
                elevatorSubsystem, RobotConstants.kElevatorStowSetpoint));
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

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Blue)
            .withProperties(Map.of("colorWhenFalse", "blue", "colorWhenTrue", "red"))
            .withSize(2, 2)
            .withPosition(0, 0);

    Shuffleboard.getTab("Match")
        .addString("Score Side", () -> robotStateSubsystem.getScoreSide().name())
        .withPosition(5, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addBoolean("Auto vs. Manual", () -> robotStateSubsystem.getIsAutoPlacing())
        .withPosition(3, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addBoolean("Has Coral", () -> robotStateSubsystem.hasCoral())
        .withPosition(4, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addString("Coral Location", () -> robotStateSubsystem.getCoralLoc().name())
        .withPosition(6, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addBoolean("Has Algae", () -> robotStateSubsystem.hasAlgae())
        .withPosition(7, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addString("Coral Level", () -> robotStateSubsystem.getCoralLevel().name())
        .withPosition(5, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addString("Algae Level", () -> robotStateSubsystem.getAlgaeHeight().name())
        .withPosition(2, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .addString("AutoSwitchPos", () -> autoSwitch.getSwitchPos())
        .withSize(1, 1)
        .withPosition(3, 0);
    Shuffleboard.getTab("Match")
        .add("ToggleVirtualSwitch", new ToggleVirtualSwitchCommand(autoSwitch))
        .withSize(1, 1)
        .withPosition(4, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Is VirtualSwitch Used", () -> autoSwitch.isUseVirtualSwitch())
        .withSize(1, 1)
        .withPosition(5, 0);
    Shuffleboard.getTab("Match")
        .add("VirtualAutoSwitch", autoSwitch.getSendableChooser())
        .withSize(2, 1)
        .withPosition(6, 0);

    Shuffleboard.getTab("Match")
        .add("Zero Biscuit", new ZeroBiscuitCommand(biscuitSubsystem))
        .withSize(1, 1)
        .withPosition(8, 0);

    Shuffleboard.getTab("Match")
        .addDouble("Climb Pos", () -> climbSubsystem.getPosition().in(Rotations))
        .withSize(1, 1)
        .withPosition(9, 1);

    // Shuffleboard.getTab("Match")
    // .addBoolean(
    // "Cams Connected",
    // () -> visionSubsystem.isCameraConnected(0) && visionSubsystem.isCameraConnected(1))
    // .withSize(1, 1)
    // .withPosition(4, 0);
  }

  private void configurePitDashboard() {
    Shuffleboard.getTab("Pit")
        .add("Stop All Moving Parts", new StopAllAxisCommand(robotStateSubsystem))
        .withPosition(1, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Pit")
        .add("Zero Elevator", new ZeroElevatorCommand(elevatorSubsystem))
        .withPosition(2, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Pit")
        .add("Lock Wheels", new lockwheelscommand(driveSubsystem))
        .withPosition(3, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Pit")
        .add("Set Azmuth Velocity 20%", new SpinUpWheelsCommand(driveSubsystem))
        .withPosition(4, 1)
        .withSize(1, 1);

    Shuffleboard.getTab("Pit")
        .add("Stop Azimuths", new StopAllAxisCommand(robotStateSubsystem).ignoringDisable(true))
        .withPosition(5, 1)
        .withSize(1, 1);
  }

  public void configTestDash() {
    Shuffleboard.getTab("Test")
        .add(
            "Turn Off Vision Updates",
            new SetVisionUpdatesCommand(visionSubsystem, false).ignoringDisable(true))
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Test")
        .add(
            "Turn On Vision Updates",
            new SetVisionUpdatesCommand(visionSubsystem, true).ignoringDisable(true))
        .withPosition(1, 0)
        .withSize(1, 1);

    // Shuffleboard.getTab("Test")
    //     .add("Start Auton", nonProcessorShallowAutonCommand)
    //     .withPosition(3, 0)
    //     .withSize(1, 1);

    // Shuffleboard.getTab("Test")
    //     .add(
    //         "reAssign Alliance",
    //         new InstantCommand(() -> nonProcessorShallowAutonCommand.reassignAlliance())
    //             .ignoringDisable(true))
    //     .withPosition(4, 0)
    //     .withSize(1, 1);

    Shuffleboard.getTab("Test")
        .add("Zero Wheels", new InstantCommand(() -> driveSubsystem.lockZero(), driveSubsystem))
        .withPosition(5, 0)
        .withSize(1, 1);

    Shuffleboard.getTab("Test")
        .add("Start Next Path", new InstantCommand(() -> pathHandler.setProceedToNext(true)))
        .withPosition(6, 0)
        .withSize(1, 1);

    Shuffleboard.getTab("Test")
        .add("Set isAuto True", new InstantCommand(() -> robotStateSubsystem.setIsAuto(true)))
        .withPosition(7, 0)
        .withSize(1, 1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void stopTagAlign() {
    tagAlignSubsystem.terminate();
  }

  public void setIsAuto(boolean isAuto) {
    robotStateSubsystem.setIsAuto(isAuto);
  }

  public void setIsAutoPlacing(boolean isAutoPlacing) {
    robotStateSubsystem.setIsAutoPlacing(isAutoPlacing);
  }

  public void setScoringSide(ScoreSide side) {
    robotStateSubsystem.setScoreSide(side);
  }

  public void stow() {
    robotStateSubsystem.toStow();
  }

  public AutoSwitch getAutoSwitch() {
    return this.autoSwitch;
  }

  public void disableNoMotionCal() {
    if (RobotConstants.isComp) {
      swerve.disableNoMotionCal();
    } else {
      protoSwerve.disableNoMotionCal();
    }
  }
}
