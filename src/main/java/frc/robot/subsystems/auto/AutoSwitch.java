package frc.robot.subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.AutoCommandInterface;
import frc.robot.commands.auton.DefaultAutonCommand;
import frc.robot.commands.auton.IRIAutonCommand;
import frc.robot.commands.auton.MiddleBargeAutonCommand;
import frc.robot.commands.auton.NonProcessorShallowAutonCommand;
import frc.robot.commands.auton.NonProcessorShallowSlowAutonCommand;
import frc.robot.commands.auton.ProcessorShallowAutonCommand;
import frc.robot.commands.auton.ProcessorShallowSlowAutonCommand;
import frc.robot.commands.auton.StealAlgaeImmediately;
import frc.robot.commands.auton.StealOneAlgeaAutonCommand;
import frc.robot.commands.auton.StealOneAlgeaNoSuperCycleAutonCommand;
import frc.robot.commands.auton.StealTwoAlgeaNoSuperCycleAutonCommand;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.PathHandlerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pathHandler.PathHandler;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch extends MeasurableSubsystem {
  public Logger logger = LoggerFactory.getLogger(AutoSwitch.class);

  private RobotStateSubsystem robotStateSubsystem;
  private AlgaeSubsystem algaeSubsystem;
  private BattMonSubsystem battMonSubsystem;
  private BiscuitSubsystem biscuitSubsystem;
  private ClimbSubsystem climbSubsystem;
  private CoralSubsystem coralSubsystem;
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private LEDSubsystem ledSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private VisionSubsystem visionSubsystem;
  private PathHandler pathHandler;

  public boolean useVirtualSwitch;
  private static SendableChooser<Integer> sendableChooser = new SendableChooser<>();
  private AutoCommandInterface defaultCommand;
  private AutoCommandInterface autoCommand;
  private final AutonSwitch autoSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int curAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;

  public AutoSwitch(
      RobotStateSubsystem robotStateSubsystem,
      AlgaeSubsystem algaeSubsystem,
      BattMonSubsystem battMonSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      ClimbSubsystem climbSubsystem,
      CoralSubsystem coralSubsystem,
      DriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      FunnelSubsystem funnelSubsystem,
      LEDSubsystem ledSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem,
      PathHandler pathHandler) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.algaeSubsystem = algaeSubsystem;
    this.battMonSubsystem = battMonSubsystem;
    this.biscuitSubsystem = biscuitSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.pathHandler = pathHandler;

    for (int i = RobotConstants.kMinAutoSwitchID; i <= RobotConstants.kMaxAutoSwitchID; i++) {
      switchInputs.add(new DigitalInput(i));
    }
    autoSwitch = new AutonSwitch(switchInputs);

    configSendableChooser();

    defaultCommand =
        new DefaultAutonCommand(
            robotStateSubsystem,
            driveSubsystem,
            elevatorSubsystem,
            "defaultAuton",
            new Pose2d(7.1008875, 7.2570308, Rotation2d.fromDegrees(180)));
  }

  public void checkSwitch() {
    if (hasSwitchChanged()) {
      logger.info("Initializing Auto Switch Position: {}", String.format("%02X", curAutoSwitchPos));
      autoCommand = getAutoCommand(curAutoSwitchPos);
      autoCommand.reassignAlliance();
    }
  }

  public void resetSwitchPos() {
    if (curAutoSwitchPos == -1) {
      logger.info("Reset Auto Switch");
    }
    curAutoSwitchPos = -1;
  }

  public AutoCommandInterface getAutoCommand() {
    if (autoCommand == null) {
      return defaultCommand;
    } else return this.autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = useVirtualSwitch ? sendableChooser.getSelected() : autoSwitch.position();

    if (switchPos != newAutoSwitchPos) {
      autoSwitchStableCounts = 0;
      newAutoSwitchPos = switchPos;
    } else autoSwitchStableCounts++;

    if (autoSwitchStableCounts > AutonConstants.kSwitchStableCounts
        && curAutoSwitchPos != newAutoSwitchPos) {
      changed = true;
      curAutoSwitchPos = newAutoSwitchPos;
    }

    return changed;
  }

  private AutoCommandInterface getAutoCommand(int switchPos) {
    switch (switchPos) {
      case 0x00 -> {
        return new NonProcessorShallowSlowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startToJSlow",
            new ArrayList<Character>(Arrays.asList('k', 'l')),
            new ArrayList<ScoringLevel>(Arrays.asList(ScoringLevel.L4, ScoringLevel.L4)),
            new ArrayList<>(
                Arrays.asList(
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance)),
            new ArrayList<>(Arrays.asList(-0.0175, 0.0, 0.0)),
            'j',
            false,
            AutonConstants.kNonProcessorShallow);
      }
        /*case 0x01 -> {
          return new NonProcessorShallowAutonCommand(
              driveSubsystem,
              pathHandler,
              robotStateSubsystem,
              algaeSubsystem,
              biscuitSubsystem,
              coralSubsystem,
              elevatorSubsystem,
              tagAlignSubsystem,
              visionSubsystem,
              "startToJ",
              new ArrayList<Character>(Arrays.asList('k', 'l', 'i')),
              new ArrayList<ScoringLevel>(
                  Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L4)),
              'j',
              false,
              false,
              AutonConstants.kNonProcessorShallow);
        }
        case 0x02 -> {
          return new NonProcessorShallowAutonCommand(
              driveSubsystem,
              pathHandler,
              robotStateSubsystem,
              algaeSubsystem,
              biscuitSubsystem,
              coralSubsystem,
              elevatorSubsystem,
              tagAlignSubsystem,
              visionSubsystem,
              "startToJ",
              new ArrayList<Character>(Arrays.asList('k', 'l', 'a')),
              new ArrayList<ScoringLevel>(
                  Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L4)),
              'j',
              false,
              false,
              AutonConstants.kNonProcessorShallow);
        }
        case 0x03 -> {
          return new NonProcessorDeepAutonCommand(
              driveSubsystem,
              pathHandler,
              robotStateSubsystem,
              algaeSubsystem,
              biscuitSubsystem,
              coralSubsystem,
              elevatorSubsystem,
              tagAlignSubsystem,
              visionSubsystem,
              "startDeepToK",
              new ArrayList<Character>(Arrays.asList('l', 'a', 'b')),
              new ArrayList<ScoringLevel>(
                  Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L4)),
              'k',
              true,
              false,
              AutonConstants.kNonProcessorDeep);
        }*/

      case 0x01 -> {
        return new NonProcessorShallowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startToJ",
            new ArrayList<Character>(Arrays.asList('k', 'l', 'k')),
            new ArrayList<ScoringLevel>(
                Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L3)),
            new ArrayList<>(Arrays.asList(2.1, 2.1, 2.1)),
            new ArrayList<>(Arrays.asList(-0.0175, 0.0, 0.0, 0.0)),
            'j',
            false,
            true,
            AutonConstants.kNonProcessorMid);
      }

      case 0x02 -> {
        return new NonProcessorShallowSlowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "midStartToJSlow",
            new ArrayList<Character>(Arrays.asList('k', 'l')),
            new ArrayList<ScoringLevel>(Arrays.asList(ScoringLevel.L4, ScoringLevel.L4)),
            new ArrayList<>(
                Arrays.asList(
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance)),
            new ArrayList<>(Arrays.asList(-0.0075, -0.0075, -0.0075)),
            'j',
            false,
            AutonConstants.kNonProcessorMid);
      }

      case 0x03 -> {
        return new IRIAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            new ArrayList<String>(Arrays.asList("deepBargeToE", "bargeToG", null)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<String>(Arrays.asList("EToBarge", "GToBarge", "bargeAway")),
            new ArrayList<>(Arrays.asList(ScoringLevel.L3, ScoringLevel.L2, ScoringLevel.L3)),
            AutonConstants.kDeepBarge);
      }

        // case 0x03 -> {
        //   return new NonProcessorShallowSlowAutonCommand(
        //       driveSubsystem,
        //       pathHandler,
        //       robotStateSubsystem,
        //       algaeSubsystem,
        //       biscuitSubsystem,
        //       coralSubsystem,
        //       elevatorSubsystem,
        //       tagAlignSubsystem,
        //       visionSubsystem,
        //       "startToJSlow",
        //       new ArrayList<Character>(Arrays.asList('k', 'l')),
        //       new ArrayList<ScoringLevel>(Arrays.asList(ScoringLevel.L4, ScoringLevel.L4)),
        //       new ArrayList<>(
        //           Arrays.asList(
        //               PathHandlerConstants.kLoadLightDistance,
        //               PathHandlerConstants.kLoadLightDistance,
        //               PathHandlerConstants.kLoadLightDistance)),
        //       new ArrayList<>(Arrays.asList(-0.0175, 0.0, 0.0)),
        //       'j',
        //       true,
        //       AutonConstants.kNonProcessorMid);
        // }

        // case 0x04 -> {
        //   return new NonProcessorMediumCycleAutonCommand(
        //       driveSubsystem,
        //       pathHandler,
        //       robotStateSubsystem,
        //       algaeSubsystem,
        //       biscuitSubsystem,
        //       coralSubsystem,
        //       elevatorSubsystem,
        //       tagAlignSubsystem,
        //       visionSubsystem,
        //       "midStartToJSlow",
        //       new ArrayList<Character>(Arrays.asList('l', 'k')),
        //       new ArrayList<ScoringLevel>(
        //           Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L4)),
        //       new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0)),
        //       new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0)),
        //       'j',
        //       false,
        //       true,
        //       AutonConstants.kNonProcessorMid);
        // }

      case 0x10 -> {
        return new MiddleBargeAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            new ArrayList<String>(Arrays.asList("startBargeToG", "bargeToI", "bargeToE")),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<String>(Arrays.asList("GToBarge", "IToBarge", "EToNearBarge")),
            new ArrayList<>(Arrays.asList(ScoringLevel.L2, ScoringLevel.L3, ScoringLevel.L3)),
            AutonConstants.kMiddleBargeStart,
            0.5);
      }

      case 0x11 -> {
        return new MiddleBargeAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            new ArrayList<String>(Arrays.asList("startBargeToG", "bargeToE", "bargeToI")),
            new ArrayList<Double>(Arrays.asList(-0.00475, 0.0, 0.0)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<String>(Arrays.asList("GToBarge", "EToBarge", "IToNearBarge")),
            new ArrayList<>(Arrays.asList(ScoringLevel.L2, ScoringLevel.L3, ScoringLevel.L3)),
            AutonConstants.kMiddleBargeStart,
            0.0);
      }

      case 0x12 -> {
        return new MiddleBargeAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            new ArrayList<String>(Arrays.asList("startBargeToG", "bargeToI", null)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<String>(Arrays.asList("GToBarge", "IToBarge", "bargeAway")),
            new ArrayList<>(Arrays.asList(ScoringLevel.L2, ScoringLevel.L3, ScoringLevel.L2)),
            AutonConstants.kMiddleBargeStart,
            2.0);
      }

      case 0x13 -> {
        return new MiddleBargeAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            new ArrayList<String>(Arrays.asList("startBargeToG", "bargeToE", null)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0)),
            new ArrayList<String>(Arrays.asList("GToBarge", "EToBarge", "bargeAway")),
            new ArrayList<>(Arrays.asList(ScoringLevel.L2, ScoringLevel.L3, ScoringLevel.L2)),
            AutonConstants.kMiddleBargeStart,
            2.0);
      }

      case 0x14 -> {
        return new StealOneAlgeaAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startBargeToG",
            "GToBarge",
            "bargeToOppE",
            "OppEToOppbarge",
            ScoringLevel.L3,
            new Pose2d(7.1, 3.7209, Rotation2d.fromRadians(3.14159)));
      }

      case 0x15 -> {
        return new StealAlgaeImmediately(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startBargeToOppE",
            "OppEToOppbarge",
            ScoringLevel.L3,
            new Pose2d(7.1, 3.7209, Rotation2d.fromRadians(3.14159)));
      }

      case 0x16 -> {
        return new StealOneAlgeaNoSuperCycleAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startHToH",
            "HToOppE",
            "OppEToOppbarge",
            ScoringLevel.L3,
            new Pose2d(7.1008875, 4.0509, Rotation2d.fromDegrees(180.0)));
      }

      case 0x17 -> {
        return new StealTwoAlgeaNoSuperCycleAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startHToH",
            "HToOppE",
            "OppEToOppbarge",
            "OppBargeToOppG",
            "OppGToOppBarge",
            ScoringLevel.L3,
            ScoringLevel.L2,
            new Pose2d(7.1008875, 4.0509, Rotation2d.fromDegrees(180.0)));
      }

      case 0x20 -> {
        return new ProcessorShallowSlowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startPToESlow",
            new ArrayList<Character>(Arrays.asList('d', 'c')),
            new ArrayList<ScoringLevel>(Arrays.asList(ScoringLevel.L4, ScoringLevel.L4)),
            new ArrayList<>(
                Arrays.asList(
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance)),
            new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0)),
            'e',
            true,
            AutonConstants.kProcessorShallow);
      }

      case 0x21 -> {
        return new ProcessorShallowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "startPToE",
            new ArrayList<Character>(Arrays.asList('d', 'c', 'c')),
            new ArrayList<ScoringLevel>(
                Arrays.asList(ScoringLevel.L4, ScoringLevel.L4, ScoringLevel.L3)),
            new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0)),
            new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0)),
            'e',
            true,
            true,
            AutonConstants.kProcessorMid);
      }

      case 0x22 -> {
        return new ProcessorShallowSlowAutonCommand(
            driveSubsystem,
            pathHandler,
            robotStateSubsystem,
            algaeSubsystem,
            biscuitSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            tagAlignSubsystem,
            visionSubsystem,
            "midStartPToESlow",
            new ArrayList<Character>(Arrays.asList('d', 'c')),
            new ArrayList<ScoringLevel>(Arrays.asList(ScoringLevel.L4, ScoringLevel.L4)),
            new ArrayList<>(
                Arrays.asList(
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance,
                    PathHandlerConstants.kLoadLightDistance)),
            new ArrayList<>(Arrays.asList(-0.0075, -0.0075, -0.0075)),
            'e',
            true,
            AutonConstants.kProcessorMid);
      }

      default -> {
        String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return defaultCommand;
      }
    }
  }

  private void configSendableChooser() {
    sendableChooser.addOption("00 nonProcessor side, on j,k,l", 0x00);
    sendableChooser.addOption("20 Processor side, on e,d,c", 0x20);
    sendableChooser.setDefaultOption("30 Do Nothing", 0x30);
    SmartDashboard.putData("Auto Mode", sendableChooser);
  }

  public void toggleVirtualSwitch() {
    logger.info("toggledSwitch:function");
    if (useVirtualSwitch) {
      useVirtualSwitch = false;
    } else {
      useVirtualSwitch = true;
    }
    // useVirtualSwitch = useVirtualSwitch ? false : true;
  }

  public SendableChooser<Integer> getSendableChooser() {
    return sendableChooser;
  }

  public boolean isUseVirtualSwitch() {
    return useVirtualSwitch;
  }

  public String getSwitchPos() {
    return Integer.toHexString(curAutoSwitchPos);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("usingVirtualSwitch", () -> this.useVirtualSwitch ? 1.0 : 0.0));
  }
}
