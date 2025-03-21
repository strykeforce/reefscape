package frc.robot.subsystems.robotState;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbAlignSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDStates;
import frc.robot.subsystems.led.LEDSubsystem.PlaceStates;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private org.slf4j.Logger logger = LoggerFactory.getLogger(this.getClass());

  private Alliance allianceColor = Alliance.Blue;

  private AlgaeSubsystem algaeSubsystem;
  private BattMonSubsystem battMonSubsystem;
  private BiscuitSubsystem biscuitSubsystem;
  private ClimbSubsystem climbSubsystem;
  private ClimbAlignSubsystem climbAlignSubsystem;
  private CoralSubsystem coralSubsystem;
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private LEDSubsystem ledSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private VisionSubsystem visionSubsystem;

  private RobotStates curState = RobotStates.IDLE;
  private RobotStates nextState;
  private RobotStates futureState;

  private ScoringLevel scoringLevel = ScoringLevel.L4;
  private ScoringLevel currentLevel = ScoringLevel.L4;
  private ScoreSide scoreSide = ScoreSide.LEFT;
  private AlgaeHeight algaeHeight = AlgaeHeight.LOW;
  private AlgaeHeight currentAlgaeHeight = AlgaeHeight.LOW;
  private CoralLoc coralLoc = CoralLoc.NONE;
  private Pose2d processorReleasePose;

  private boolean isAutoPlacing = false;
  private boolean getAlgaeOnCycle = false;
  private boolean isCurrentLimiting = false;
  private boolean isAuto = false;
  private boolean isEjectingAlgae = false;
  private boolean isBargeSafe = true;
  private boolean isAutoReadyForEject = false;
  private boolean prestagingForAlgae = false;
  private boolean hasElevatorAutoCoralPrestaged = false;

  private Timer scoringTimer = new Timer();

  public RobotStateSubsystem(
      AlgaeSubsystem algaeSubsystem,
      BattMonSubsystem battMonSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      ClimbSubsystem climbSubsystem,
      ClimbAlignSubsystem climbAlignSubsystem,
      CoralSubsystem coralSubsystem,
      DriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      FunnelSubsystem funnelSubsystem,
      LEDSubsystem ledSubsystem,
      TagAlignSubsystem tagAlignSubsystem,
      VisionSubsystem visionSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.battMonSubsystem = battMonSubsystem;
    this.biscuitSubsystem = biscuitSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.climbAlignSubsystem = climbAlignSubsystem;
    this.coralSubsystem = coralSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.visionSubsystem = visionSubsystem;

    ledSubsystem.setState(LEDStates.NORMAL);
  }

  public RobotStates getState() {
    return curState;
  }

  public RobotStates getNextState() {
    return nextState;
  }

  public CoralLoc getCoralLoc() {
    return coralLoc;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public ScoringLevel getAlgaeLevel() {
    return (tagAlignSubsystem.computeHexant(allianceColor) % 2) == 0
        ? ScoringLevel.L3
        : ScoringLevel.L2;
  }

  public ScoringLevel getCoralLevel() {
    return scoringLevel;
  }

  public ScoreSide getScoreSide() {
    return scoreSide;
  }

  public AlgaeHeight getAlgaeHeight() {
    return algaeHeight;
  }

  public boolean getIsBargeSafe() {
    return isBargeSafe;
  }

  public boolean getGetAlgaeOnCycle() {
    return getAlgaeOnCycle;
  }

  public boolean getIsAuto() {
    return isAuto;
  }

  public boolean getIsAutoPlacing() {
    return isAutoPlacing;
  }

  public boolean hasCoral() {
    return coralSubsystem.hasCoral();
  }

  public boolean hasCoralAuton() {
    return funnelSubsystem.hasCoral();
  }

  public boolean hasAlgae() {
    return algaeSubsystem.hasAlgae();
  }

  public boolean safeMoveElevator() {
    return !(coralSubsystem.getState() != CoralState.HAS_CORAL && funnelSubsystem.hasCoral());
  }

  public void startupSequence() {
    if (curState == RobotStates.IDLE) {
      elevatorSubsystem.zero();
      setState(RobotStates.STARTUP);
    }
  }

  public void setState(RobotStates robotState, boolean transfer) {
    if (curState != robotState) {
      if (transfer) {
        logger.info("TRANSFER ({} -> {})", curState, robotState);
        nextState = robotState;
        curState = RobotStates.TRANSFER;
      } else {
        logger.info("{} -> {}", this.curState, robotState);
        curState = nextState = robotState;
      }
    }
  }

  private void setState(RobotStates robotState) {
    setState(robotState, false);
  }

  public void setAllianceColor(Alliance alliance) {
    allianceColor = alliance;
    logger.info("Change color to: {}", allianceColor);
  }

  public void setScoringLevel(ScoringLevel scoringLevel) {
    this.scoringLevel = scoringLevel;
    ledSubsystem.setLevelLights(scoringLevel);
  }

  public void setScoreSide(ScoreSide scoreSide) {
    this.scoreSide = scoreSide;
    if (!isAutoPlacing) ledSubsystem.setPlaceLights(PlaceStates.MANUAL);
    else if (scoreSide == ScoreSide.LEFT) ledSubsystem.setPlaceLights(PlaceStates.LEFT);
    else ledSubsystem.setPlaceLights(PlaceStates.RIGHT);
  }

  public void setAlgaeHeight(AlgaeHeight algaeHeight) {
    this.algaeHeight = algaeHeight;
  }

  public void toggleAlgaeHeight() {
    algaeHeight = algaeHeight == AlgaeHeight.LOW ? AlgaeHeight.HIGH : AlgaeHeight.LOW;
  }

  public void setIsAutoPlacing(boolean isAutoPlacing) {
    this.isAutoPlacing = isAutoPlacing;
    if (!isAutoPlacing) ledSubsystem.setPlaceLights(PlaceStates.MANUAL);
    else if (scoreSide == ScoreSide.LEFT) ledSubsystem.setPlaceLights(PlaceStates.LEFT);
    else ledSubsystem.setPlaceLights(PlaceStates.RIGHT);
  }

  public void setGetAlgaeOnCycle(boolean getAlgaeOnCycle) {
    this.getAlgaeOnCycle = getAlgaeOnCycle;
    ledSubsystem.setGetAlgeaLights(getAlgaeOnCycle);
  }

  public void toggleGetAlgaeOnCycle() {
    getAlgaeOnCycle = !getAlgaeOnCycle;
    ledSubsystem.setGetAlgeaLights(getAlgaeOnCycle);
  }

  public void setCurrentLimiting(boolean isCurrentLimiting) {
    this.isCurrentLimiting = isCurrentLimiting;
    ledSubsystem.setCurrentLimiting(isCurrentLimiting);
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
    isAutoReadyForEject = false;
  }

  public void setAutoPlacingLed(boolean isAutoPlacing) {
    ledSubsystem.setAutoPlacing(isAutoPlacing);
  }

  private boolean needSafeAlgaeTransfer(RobotStates nextState) {
    if (algaeSubsystem.hasAlgae()) {
      switch (curState) {
        case FLOOR_ALGAE, MIC_ALGAE, PROCESSOR_ALGAE, BARGE_ALGAE, HP_ALGAE, INTERRUPTED -> {
          switch (nextState) {
              // These are all the states that could possibly be entered that are also possibly
              // dangerous
              // Each futureState must be handled in STOW
            case REEF_ALIGN_ALGAE, REEF_ALIGN_CORAL, PREP_CLIMB -> {
              futureState = nextState;
              toStowSafe();
              return true;
            }
            default -> {}
          }
        }
        default -> {}
      }

      switch (curState) {
        case REEF_ALIGN_ALGAE, REEF_ALIGN_CORAL, REMOVE_ALGAE, PLACE_CORAL, INTERRUPTED -> {
          switch (nextState) {
              // These are all the states that could possibly be entered that are also
              // dangerous
              // Each futureState must be handled in STOW
            case FLOOR_ALGAE, MIC_ALGAE, PROCESSOR_ALGAE, BARGE_ALGAE, HP_ALGAE, PREP_CLIMB -> {
              futureState = nextState;
              toStowSafe();
              return true;
            }
            default -> {}
          }
        }
        default -> {}
      }
    }

    return false;
  }

  public void toStow() {
    visionSubsystem.setYawUpdateCamera(-1);
    biscuitSubsystem.setIsRemovingAlgae(false);
    driveSubsystem.removeDriveMultiplier();
    driveSubsystem.setIgnoreSticks(false);

    if (biscuitSubsystem.isSafeToStow()) {
      biscuitSubsystem.setPosition(RobotConstants.kStowSetpoint, hasAlgae());
      elevatorSubsystem.setPosition(RobotConstants.kElevatorStowSetpoint);
      algaeSubsystem.hold();

      setState(RobotStates.TO_STOW);
    } else {
      toStowSafe();
    }
  }

  public void toStowSafe() {
    biscuitSubsystem.setPosition(RobotConstants.kStowSetpoint, hasAlgae());
    driveSubsystem.removeDriveMultiplier();
    driveSubsystem.setIgnoreSticks(false);
    algaeSubsystem.hold();

    setState(RobotStates.TO_STOW_SAFE);
  }

  public void toStowSequential() {
    biscuitSubsystem.setPosition(RobotConstants.kStowSetpoint, hasAlgae());
    driveSubsystem.removeDriveMultiplier();
    driveSubsystem.setIgnoreSticks(false);
    algaeSubsystem.hold();

    setState(RobotStates.TO_STOW_SEQUENTIAL);
  }

  public void toAutonPrestage() {
    coralLoc = CoralLoc.CORAL;
    biscuitSubsystem.setPosition(RobotConstants.kPrestageSetpoint, hasAlgae());
    elevatorSubsystem.setPosition(ElevatorConstants.kAutoPrestageSetpoint);
    funnelSubsystem.stopMotor();

    setState(RobotStates.PRESTAGE, true);
  }

  public void toFunnelLoad() {
    biscuitSubsystem.setPosition(RobotConstants.kFunnelSetpoint, hasAlgae());
    coralSubsystem.intake();
    elevatorSubsystem.setPosition(RobotConstants.kElevatorFunnelSetpoint);

    setState(RobotStates.FUNNEL_LOAD, true);
  }

  private void toPrestage() {
    biscuitSubsystem.setPosition(RobotConstants.kPrestageSetpoint, hasAlgae());

    prestagingForAlgae = getAlgaeOnCycle;

    if (getAlgaeOnCycle) {
      elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
    } else {
      elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);
    }

    funnelSubsystem.stopMotor();

    setState(RobotStates.PRESTAGE, true);
  }

  public void toPrepCoral() {
    if (curState == RobotStates.REEF_ALIGN_CORAL) {
      if (isAuto) isAutoReadyForEject = true;
      if (elevatorSubsystem.isFinished()) {
        toPlaceCoral();
      }
    } else {
      toReefAlign(getAlgaeOnCycle, isAutoPlacing);
    }
  }

  public void toPlaceCoralAuto() {
    isAutoReadyForEject = true;
    if (elevatorSubsystem.isFinished()) toPlaceCoral();
    else toReefAlign(getAlgaeOnCycle, false);
  }

  public void toReefAlign() {
    toReefAlign(getAlgaeOnCycle, true);
  }

  public void toReefAlignAlgaeAuto() {
    toReefAlign(true, true);
  }

  private void toReefAlign(boolean getAlgae, boolean drive) {
    if (!safeMoveElevator()) {
      logger.info("Elevator movement is dangerous!");
      return;
    }
    boolean wantAlgae = getAlgae || !coralSubsystem.hasCoral();
    if ((hasAlgae() && scoreSide == ScoreSide.RIGHT && drive) || (hasAlgae() && !hasCoral())) {
      setState(RobotStates.STOW);
    }
    if (drive) {
      tagAlignSubsystem.start(
          allianceColor,
          scoreSide == ScoreSide.LEFT
              || !coralSubsystem.hasCoral()
              || (wantAlgae && getAlgaeOnCycle),
          wantAlgae);
      setAutoPlacingLed(true);
      setState(RobotStates.REEF_ALIGN);
    }

    boolean algaeSafe =
        tagAlignSubsystem.getCurRadius(allianceColor) > TagServoingConstants.kAlgaeStopXDriveRadius
            || tagAlignSubsystem.getState() != TagAlignStates.DRIVE;

    boolean ignoreCoralScoring = isAutoPlacing && getAlgaeOnCycle && scoreSide == ScoreSide.RIGHT;

    if (wantAlgae && !algaeSafe) {
      algaeSubsystem.intake();
      biscuitSubsystem.setPosition(RobotConstants.kPrestageAlgaeSetpoint, hasAlgae());

      switch (getAlgaeLevel()) {
        case L2 -> {
          elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
        }
        case L3 -> {
          elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeSetpoint);
        }
        default -> logger.error("Invalid algae level: {}", getAlgaeLevel());
      }
    }

    if (wantAlgae
        && algaeSafe
        && (scoreSide == ScoreSide.LEFT || !isAutoPlacing || ignoreCoralScoring)
        && !algaeSubsystem.hasAlgae()) {
      if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_ALGAE)) {
        return;
      }
      // Intentionally over-writes state for auto-pickup algae from REEF_ALIGN to REEF_ALIGN_ALGAE
      setState(RobotStates.REEF_ALIGN_ALGAE, true);

      algaeSubsystem.intake();

      switch (getAlgaeLevel()) {
        case L2 -> {
          biscuitSubsystem.setPosition(RobotConstants.kL2AlgaeSetpoint, hasAlgae());
          elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
        }
        case L3 -> {
          biscuitSubsystem.setPosition(RobotConstants.kL3AlgaeSetpoint, hasAlgae());
          elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeSetpoint);
        }
        default -> logger.error("Invalid algae level: {}", getAlgaeLevel());
      }
    } else if (!drive) {
      if (!coralSubsystem.hasCoral() || ignoreCoralScoring) {
        toStow();
        return;
      } else {
        if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_CORAL)) {
          return;
        }

        if (!isAutoPlacing
            || (tagAlignSubsystem.yErrorSmall()
                && tagAlignSubsystem.getCurRadius(allianceColor)
                    < RobotStateConstants.kElevatorWaitRadius)) {
          currentLevel = scoringLevel;
          switch (scoringLevel) {
            case L1 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL1CoralSetpoint, hasAlgae());
              elevatorSubsystem.setPosition(ElevatorConstants.kL1CoralSetpoint);
            }
            case L2 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL2CoralSetpoint, hasAlgae());
              elevatorSubsystem.setPosition(ElevatorConstants.kL2CoralSetpoint);
            }
            case L3 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL3CoralSetpoint, hasAlgae());
              elevatorSubsystem.setPosition(ElevatorConstants.kL3CoralSetpoint);
            }
            case L4 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL4CoralSetpoint, hasAlgae());
              elevatorSubsystem.setPosition(ElevatorConstants.kL4CoralSetpoint);
            }
          }
          hasElevatorAutoCoralPrestaged = true;
        } else {
          hasElevatorAutoCoralPrestaged = false;
        }

        setState(RobotStates.REEF_ALIGN_CORAL, true);
      }
    }
  }

  public void finishAuto() {
    toReefAlign(false, false);
    setState(RobotStates.FINISH_AUTO);
  }

  public void toPlaceCoral() {
    if (isAutoPlacing) {
      visionSubsystem.setYawUpdateCamera(scoreSide == ScoreSide.LEFT ? 2 : 0);
    }
    coralSubsystem.eject(scoringLevel);
    funnelSubsystem.clearCoral();
    isAutoReadyForEject = false;
    scoringTimer.stop();
    scoringTimer.reset();
    scoringTimer.start();
    setState(RobotStates.PLACE_CORAL);
  }

  public void toAlgaeFloorPickup() {
    if (!safeMoveElevator()) {
      logger.info("Elevator movement is dangerous!");
      return;
    }

    algaeSubsystem.intake();
    currentAlgaeHeight = algaeHeight;

    switch (algaeHeight) {
      case LOW -> {
        if (needSafeAlgaeTransfer(RobotStates.FLOOR_ALGAE)) {
          return;
        }

        biscuitSubsystem.setPosition(RobotConstants.kFloorAlgaeSetpoint, hasAlgae());
        elevatorSubsystem.setPosition(ElevatorConstants.kFloorAlgaeSetpoint);

        setState(RobotStates.FLOOR_ALGAE, true);
      }
      case HIGH -> {
        if (needSafeAlgaeTransfer(RobotStates.MIC_ALGAE)) {
          return;
        }

        biscuitSubsystem.setPosition(RobotConstants.kMicAlgaeSetpoint, hasAlgae());
        elevatorSubsystem.setPosition(ElevatorConstants.kMicAlgaeSetpoint);

        setState(RobotStates.MIC_ALGAE, true);
      }
    }
  }

  public void toScoreAlgae() {
    currentAlgaeHeight = algaeHeight;
    if (curState == RobotStates.BARGE_ALGAE && algaeHeight == AlgaeHeight.LOW) {
      futureState = RobotStates.PROCESSOR_ALGAE;
      toStowSafe();
    } else if (curState == RobotStates.PROCESSOR_ALGAE && algaeHeight == AlgaeHeight.HIGH) {
      futureState = RobotStates.BARGE_ALGAE;
      toStowSafe();
    }

    if (!safeMoveElevator()) {
      logger.info("Elevator movement is dangerous!");
      return;
    }

    switch (algaeHeight) {
      case LOW -> {
        toProcessor();
      }
      case HIGH -> {
        if (needSafeAlgaeTransfer(RobotStates.BARGE_ALGAE)) {
          return;
        }

        driveSubsystem.setDriveMultiplier(DriveConstants.kBargeScoreStickMultiplier);

        double poseX = driveSubsystem.getPoseMeters().getX();

        isBargeSafe =
            poseX > RobotStateConstants.kRedBargeSafeX
                || poseX < RobotStateConstants.kBlueBargeSafeX;

        if (isBargeSafe) {
          elevatorSubsystem.setPosition(ElevatorConstants.kBargeSetpoint);

          setState(RobotStates.TO_BARGE_ALGAE);
        }
      }
    }
  }

  public void stopAxis() {
    coralSubsystem.setSpeed(RotationsPerSecond.of(0));
    funnelSubsystem.setPercent(0);
  }

  public void releaseAlgae() {
    scoringTimer.stop();
    scoringTimer.reset();
    scoringTimer.start();
    isEjectingAlgae = true;

    processorReleasePose = driveSubsystem.getPoseMeters();
    Logger.recordOutput("RobotState/Processor Release Pose", processorReleasePose);
    driveSubsystem.removeDriveMultiplier();

    switch (algaeHeight) {
      case LOW -> {
        algaeSubsystem.scoreProcessor();
        setState(RobotStates.PROCESSOR_ALGAE);
      }
      case HIGH -> {
        algaeSubsystem.scoreBarge();
        setState(RobotStates.BARGE_ALGAE);
      }
    }
  }

  public void toHpAlgae() {
    if (!safeMoveElevator()) {
      logger.info("Elevator movement is dangerous!");
      return;
    }
    currentAlgaeHeight = algaeHeight;
    if (needSafeAlgaeTransfer(RobotStates.HP_ALGAE)) {
      return;
    }

    algaeSubsystem.intake();

    biscuitSubsystem.setPosition(RobotConstants.kHpAlgaeSetpoint, hasAlgae());
    elevatorSubsystem.setPosition(ElevatorConstants.kHpAlgaeSetpoint);

    setState(RobotStates.HP_ALGAE, true);
  }

  private void toProcessor() {
    if (!safeMoveElevator()) {
      logger.info("Elevator movement is dangerous!");
      return;
    }

    currentAlgaeHeight = algaeHeight;
    if (needSafeAlgaeTransfer(RobotStates.PROCESSOR_ALGAE)) {
      return;
    }

    biscuitSubsystem.setPosition(RobotConstants.kProcessorSetpoint, hasAlgae());
    elevatorSubsystem.setPosition(ElevatorConstants.kProcessorSetpoint);

    setState(RobotStates.PROCESSOR_ALGAE, true);
  }

  public void toInterrupted() {
    if (tagAlignSubsystem.getState() != TagAlignSubsystem.TagAlignStates.DONE) {
      tagAlignSubsystem.terminate();
      visionSubsystem.setYawUpdateCamera(-1);
      setAutoPlacingLed(false);
      driveSubsystem.setIgnoreSticks(false);
    }

    if (climbAlignSubsystem.getState() != ClimbAlignSubsystem.ClimbAlignStates.DONE) {
      climbAlignSubsystem.terminate();
      driveSubsystem.setIgnoreSticks(false);
    }

    biscuitSubsystem.setIsRemovingAlgae(false);

    biscuitSubsystem.setPosition(biscuitSubsystem.getPosition(), hasAlgae());
    elevatorSubsystem.setPosition(elevatorSubsystem.getPosition());

    setState(RobotStates.INTERRUPTED);
  }

  public void toPrepClimb() {
    if (needSafeAlgaeTransfer(RobotStates.PREP_CLIMB)) {
      return;
    }

    biscuitSubsystem.setPosition(RobotConstants.kStowSetpoint, hasAlgae());
    elevatorSubsystem.setPosition(RobotConstants.kElevatorStowSetpoint);

    climbSubsystem.prepClimb();
    driveSubsystem.prepClimb();

    setState(RobotStates.PREP_CLIMB, true);
  } // -0.30

  public void toClimb() {
    if (curState == RobotStates.PREP_CLIMB || climbSubsystem.getState() == ClimbState.PREPPED) {
      climbSubsystem.climb();
      driveSubsystem.prepClimb();

      setState(RobotStates.CLIMB, true);
    }
  }

  public boolean isElevatorFinished() {
    return elevatorSubsystem.isFinished();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("RobotState/state", curState);
    Logger.recordOutput("RobotState/hasCoral", hasCoral());
    Logger.recordOutput("RobotState/hasAlgae", hasAlgae());
    Logger.recordOutput("RobotState/scoringLevel", scoringLevel);
    Logger.recordOutput("RobotState/algaeHeight", algaeHeight);
    Logger.recordOutput("RobotState/getAlgae", getAlgaeOnCycle);
    Logger.recordOutput("RobotState/isAutoPlacing", isAutoPlacing);
    Logger.recordOutput("RobotState/scoreSide", scoreSide);
    Logger.recordOutput("RobotState/isBargeSafe", isBargeSafe);
    Logger.recordOutput("RobotState/isAuto", isAuto);
    Logger.recordOutput("RobotState/currentLimiting", isCurrentLimiting);
    Logger.recordOutput("RobotState/isEjectingAlgae", isEjectingAlgae);
    Logger.recordOutput("RobotState/isAutoReadyForEject", isAutoReadyForEject);
    Logger.recordOutput("RobotState/coralLoc", coralLoc);
    Logger.recordOutput("RobotState/alliance", allianceColor);

    switch (curState) {
      case TRANSFER -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()
            || ((nextState == RobotStates.PREP_CLIMB || nextState == RobotStates.CLIMB)
                && elevatorSubsystem.getPosition().in(Rotations)
                    < RobotStateConstants.kElevatorClimbMax)
        // && climbSubsystem.isFinished()
        ) {
          setState(nextState);
        }
      }

      case TO_STOW -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          driveSubsystem.removeDriveMultiplier();
          setState(RobotStates.STOW);
        }
      }

      case TO_STOW_SAFE -> {
        if (biscuitSubsystem.isSafeToStow()) {
          elevatorSubsystem.setPosition(RobotConstants.kElevatorStowSetpoint);
          setState(RobotStates.TO_STOW);
        }
      }

      case TO_STOW_SEQUENTIAL -> {
        if (biscuitSubsystem.isFinished()) {
          elevatorSubsystem.setPosition(RobotConstants.kElevatorStowSetpoint);
          setState(RobotStates.TO_STOW);
        }
      }

      case STOW -> {
        if (futureState != null) {
          switch (futureState) {
            case REEF_ALIGN_ALGAE -> toReefAlign(true, isAutoPlacing);
            case REEF_ALIGN_CORAL -> toReefAlign(false, isAutoPlacing);
            case HP_ALGAE -> toHpAlgae();
            case PROCESSOR_ALGAE, BARGE_ALGAE -> toScoreAlgae();
            case MIC_ALGAE, FLOOR_ALGAE -> toAlgaeFloorPickup();
            case PREP_CLIMB -> toPrepClimb();
            default -> logger.error("Unhandled future state: {}", futureState);
          }
        }
        futureState = null;

        if (!coralSubsystem.hasCoral()) {
          toFunnelLoad();
        }
      }
      case REEF_ALIGN -> {
        if (!isAutoPlacing
            || tagAlignSubsystem.getState() != TagAlignSubsystem.TagAlignStates.DRIVE) {
          toReefAlign(getAlgaeOnCycle, false);
        }
      }
      case REEF_ALIGN_ALGAE -> {
        if (algaeSubsystem.hasAlgaeSuperCycle()) {
          biscuitSubsystem.setIsRemovingAlgae(true);
          switch (getAlgaeLevel()) {
            case L2 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL2AlgaeRemovalSetpoint, true);
              elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeRemovalSetpoint);
            }
            case L3 -> {
              biscuitSubsystem.setPosition(RobotConstants.kL3AlgaeRemovalSetpoint, true);
              elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeRemovalSetpoint);
            }
            default -> logger.error("Invalid algae level: {}", getAlgaeLevel());
          }

          setState(RobotStates.REMOVE_ALGAE);
        }
      }
      case REEF_ALIGN_CORAL -> {
        if (currentLevel != scoringLevel) {
          toReefAlign(false, false);
        }
        if (!hasElevatorAutoCoralPrestaged && tagAlignSubsystem.yErrorSmall()) {
          toReefAlign(false, false);
          break;
        }
        if ((isAutoPlacing
                && tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE
                && elevatorSubsystem.isFinished())
            || (isAuto && isAutoReadyForEject)) {
          toPlaceCoral();
          isAutoReadyForEject = false;
        }
      }
      case REMOVE_ALGAE -> {
        if (elevatorSubsystem.isFinished()
            && biscuitSubsystem.getPosition().in(Rotations)
                <= RobotStateConstants.kBiscuitSuperCycleSafeThres) {
          biscuitSubsystem.setIsRemovingAlgae(false);
          if (coralSubsystem.hasCoral()
              && !(isAutoPlacing && getAlgaeOnCycle && scoreSide == ScoreSide.RIGHT)) {
            toReefAlign(false, false);
          } else {
            setAutoPlacingLed(false);
            toStowSequential();
          }
        }
      }

      case PLACE_CORAL -> {
        if (!coralSubsystem.hasCoral()
            && scoringTimer.hasElapsed(RobotStateConstants.kCoralEjectTimer)) {
          coralLoc = CoralLoc.NONE;
          visionSubsystem.setYawUpdateCamera(-1);
          setAutoPlacingLed(false);
          driveSubsystem.setIgnoreSticks(false);
          toFunnelLoad();
        } else {
          coralLoc = CoralLoc.SCORING;
        }
      }

      case FUNNEL_LOAD -> {
        if (funnelSubsystem.hasCoral()) {
          coralLoc = CoralLoc.FUNNEL;
        }
        if (elevatorSubsystem.isFinished()) {
          funnelSubsystem.startMotor();
        }
        if (funnelSubsystem.hasCoral()) {
          coralLoc = CoralLoc.TRANSFER;
          setState(RobotStates.LOADING_CORAL);
        }
      }
      case LOADING_CORAL -> {
        if (coralSubsystem.hasCoral()) {
          coralLoc = CoralLoc.CORAL;
          // biscuitSubsystem.setPosition(BiscuitConstants.kPrestageSetpoint);
          if (isAuto) {
            toAutonPrestage();
            // elevatorSubsystem.setPosition(ElevatorConstants.kAutoPrestageSetpoint);
            break;
          } else {
            toPrestage();
            // elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);
            break;
          }
          // funnelSubsystem.stopMotor();

          // setState(RobotStates.PRESTAGE, true);
        }
      }
      case PRESTAGE -> {
        if (getAlgaeOnCycle != prestagingForAlgae) {
          toPrestage();
        }
      }
      case HP_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toStowSafe();
        }
      }
      case PROCESSOR_ALGAE -> {
        if (isEjectingAlgae
            && !algaeSubsystem.hasAlgae()
            && scoringTimer.hasElapsed(RobotStateConstants.kAlgaeEjectTimer)) {
          Pose2d currentPose = driveSubsystem.getPoseMeters();
          double distanceFromRelease =
              FastMath.hypot(
                  currentPose.getX() - processorReleasePose.getX(),
                  currentPose.getY() - processorReleasePose.getY());
          Logger.recordOutput("RobotState/Processor Release Distance", distanceFromRelease);

          if (distanceFromRelease > RobotStateConstants.kProcessorStowRadius) {
            isEjectingAlgae = false;
            toStowSafe();
          }
        } else if (algaeHeight != currentAlgaeHeight) {
          toScoreAlgae();
        }
      }
      case TO_BARGE_ALGAE -> {
        if (elevatorSubsystem.isHigherThan(ElevatorConstants.kBargeHigherThan)) {
          biscuitSubsystem.setPosition(RobotConstants.kBargeSetpoint, hasAlgae());
          curState = RobotStates.BARGE_ALGAE;
        }
      }
      case BARGE_ALGAE -> {
        if (!algaeSubsystem.hasAlgae()
            && scoringTimer.hasElapsed(RobotStateConstants.kAlgaeEjectTimer)
            && isEjectingAlgae) {
          isEjectingAlgae = false;
          toStowSafe();
        } else if (algaeHeight != currentAlgaeHeight) {
          toScoreAlgae();
        }
      }
      case MIC_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toStowSafe();
        } else if (algaeHeight != currentAlgaeHeight) {
          toAlgaeFloorPickup();
        }
      }
      case FLOOR_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toStowSafe();
        } else if (algaeHeight != currentAlgaeHeight) {
          toAlgaeFloorPickup();
        }
      }
      case PREP_CLIMB -> {
        double climbPos = climbSubsystem.getPosition().in(Rotations);
        if (climbPos < RobotStateConstants.kClimbAngleSmall) {
          ledSubsystem.setState(LEDStates.TOO_CLOSE);
        } else if (climbPos > RobotStateConstants.kClimbAngleBig) {
          ledSubsystem.setState(LEDStates.TOO_FAR);
        } else {
          ledSubsystem.setState(LEDStates.GOOD);
        }
      }
      case CLIMB -> {}
      case INTERRUPTED -> {}
      case IDLE -> {}
      case STARTUP -> {
        if (elevatorSubsystem.getState() == ElevatorStates.ZEROED) toStow();
      }
      case FINISH_AUTO -> {
        if (elevatorSubsystem.isFinished()) {
          toPlaceCoral();
        }
      }
      default -> logger.error("Unhandled state: {}", curState);
    }
    ledSubsystem.setCoralLights(coralLoc);
    ledSubsystem.setAlgeaLights(hasAlgae());
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> curState.ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  public enum RobotStates {
    STOW,
    TO_STOW,
    TO_STOW_SAFE,
    TO_STOW_SEQUENTIAL,
    REEF_ALIGN,
    REEF_ALIGN_ALGAE,
    REEF_ALIGN_CORAL,
    REMOVE_ALGAE,
    PLACE_CORAL,
    FUNNEL_LOAD,
    LOADING_CORAL,
    PRESTAGE,
    HP_ALGAE,
    PROCESSOR_ALGAE,
    TO_BARGE_ALGAE,
    BARGE_ALGAE,
    MIC_ALGAE,
    FLOOR_ALGAE,
    PREP_CLIMB,
    CLIMB,
    INTERRUPTED,
    TRANSFER,
    IDLE,
    STARTUP,
    FINISH_AUTO
  }

  public enum ScoringLevel {
    L1,
    L2,
    L3,
    L4
  }

  public enum ScoreSide {
    LEFT,
    RIGHT
  }

  public enum AlgaeHeight {
    LOW,
    HIGH
  }

  public enum CoralLoc {
    FUNNEL,
    TRANSFER,
    CORAL,
    SCORING,
    NONE
  }
}
