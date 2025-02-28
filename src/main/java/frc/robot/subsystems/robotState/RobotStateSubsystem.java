package frc.robot.subsystems.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
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
  private CoralLoc coralLoc = CoralLoc.CORAL;
  private Pose2d algaeRemovalPose;

  private boolean isAutoPlacing = false;
  private boolean getAlgaeOnCycle = false;
  private boolean isCurrentLimiting = false;
  private boolean isAuto = false;
  private boolean isEjectingAlgae = false;
  private boolean isBargeSafe = true;

  private Timer scoringTimer = new Timer();

  public RobotStateSubsystem(
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
      VisionSubsystem visionSubsystem) {
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

  public ScoreSide getScoreSide() {
    return scoreSide;
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

  public boolean hasAlgae() {
    return algaeSubsystem.hasAlgae();
  }

  private void setState(RobotStates robotState, boolean transfer) {
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
  }

  public void setScoreSide(ScoreSide scoreSide) {
    this.scoreSide = scoreSide;
  }

  public void setAlgaeHeight(AlgaeHeight algaeHeight) {
    this.algaeHeight = algaeHeight;
  }

  public void toggleAlgaeHeight() {
    algaeHeight = algaeHeight == AlgaeHeight.LOW ? AlgaeHeight.HIGH : AlgaeHeight.LOW;
  }

  public void setIsAutoPlacing(boolean isAutoPlacing) {
    this.isAutoPlacing = isAutoPlacing;
  }

  public void setGetAlgaeOnCycle(boolean getAlgaeOnCycle) {
    this.getAlgaeOnCycle = getAlgaeOnCycle;
  }

  public void toggleGetAlgaeOnCycle() {
    getAlgaeOnCycle = !getAlgaeOnCycle;
  }

  public void setCurrentLimiting(boolean isCurrentLimiting) {
    this.isCurrentLimiting = isCurrentLimiting;
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
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
    if (biscuitSubsystem.isSafeToStow()) {
      biscuitSubsystem.setPosition(BiscuitConstants.kStowSetpoint);
      elevatorSubsystem.setPosition(ElevatorConstants.kStowSetpoint);
      driveSubsystem.removeDriveMultiplier();
      driveSubsystem.setIgnoreSticks(false);
      algaeSubsystem.hold();

      setState(RobotStates.TO_STOW);
    } else {
      toStowSafe();
    }
  }

  public void toStowSafe() {
    biscuitSubsystem.setPosition(BiscuitConstants.kStowSetpoint);
    driveSubsystem.removeDriveMultiplier();
    driveSubsystem.setIgnoreSticks(false);
    algaeSubsystem.hold();

    setState(RobotStates.TO_STOW_SAFE);
  }

  public void toStowSequential() {
    biscuitSubsystem.setPosition(BiscuitConstants.kStowSetpoint);
    driveSubsystem.removeDriveMultiplier();
    driveSubsystem.setIgnoreSticks(false);
    algaeSubsystem.hold();

    setState(RobotStates.TO_STOW_SEQUENTIAL);
  }

  private void toFunnelLoad() {
    biscuitSubsystem.setPosition(BiscuitConstants.kFunnelSetpoint);
    coralSubsystem.intake();
    elevatorSubsystem.setPosition(ElevatorConstants.kFunnelSetpoint);

    setState(RobotStates.FUNNEL_LOAD, true);
  }

  public void toPrepCoral() {
    if (curState == RobotStates.REEF_ALIGN_CORAL) {
      if (elevatorSubsystem.isFinished()) {
        toPlaceCoral();
      }
    } else {
      toReefAlign(getAlgaeOnCycle, isAutoPlacing);
    }
  }

  public void toReefAlign() {
    toReefAlign(getAlgaeOnCycle, true);
  }

  private void toReefAlign(boolean getAlgae, boolean drive) {
    if ((hasAlgae() && scoreSide == ScoreSide.RIGHT && drive) || (hasAlgae() && !hasCoral())) {
      setState(RobotStates.STOW);
    }
    if (drive) {
      tagAlignSubsystem.start(allianceColor, scoreSide == ScoreSide.LEFT);
      setState(RobotStates.REEF_ALIGN);
    }
    if ((getAlgae || !coralSubsystem.hasCoral())
        && (scoreSide == ScoreSide.LEFT || !isAutoPlacing)
        && !algaeSubsystem.hasAlgae()) {
      if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_ALGAE)) {
        return;
      }
      // Intentionally over-writes state for auto-pickup algae from REEF_ALIGN to REEF_ALIGN_ALGAE
      setState(RobotStates.REEF_ALIGN_ALGAE, true);

      algaeSubsystem.intake();

      switch (getAlgaeLevel()) {
        case L2 -> {
          biscuitSubsystem.setPosition(BiscuitConstants.kL2AlgaeSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
        }
        case L3 -> {
          biscuitSubsystem.setPosition(BiscuitConstants.kL3AlgaeSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeSetpoint);
        }
        default -> logger.error("Invalid algae level: {}", getAlgaeLevel());
      }
    } else if (!drive) {
      if (!coralSubsystem.hasCoral()) {
        toStow();
        return;
      } else {
        if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_CORAL)) {
          return;
        }

        currentLevel = scoringLevel;
        switch (scoringLevel) {
          case L1 -> {
            biscuitSubsystem.setPosition(BiscuitConstants.kL1CoralSetpoint);
            elevatorSubsystem.setPosition(ElevatorConstants.kL1CoralSetpoint);
          }
          case L2 -> {
            biscuitSubsystem.setPosition(BiscuitConstants.kL2CoralSetpoint);
            elevatorSubsystem.setPosition(ElevatorConstants.kL2CoralSetpoint);
          }
          case L3 -> {
            biscuitSubsystem.setPosition(BiscuitConstants.kL3CoralSetpoint);
            elevatorSubsystem.setPosition(ElevatorConstants.kL3CoralSetpoint);
          }
          case L4 -> {
            biscuitSubsystem.setPosition(BiscuitConstants.kL4CoralSetpoint);
            elevatorSubsystem.setPosition(ElevatorConstants.kL4CoralSetpoint);
          }
        }
        setState(RobotStates.REEF_ALIGN_CORAL, true);
      }
    }
  }

  public void toPlaceCoral() {
    coralSubsystem.eject();
    funnelSubsystem.clearCoral();
    scoringTimer.stop();
    scoringTimer.reset();
    scoringTimer.start();
    setState(RobotStates.PLACE_CORAL);
  }

  public void toAlgaeFloorPickup() {
    algaeSubsystem.intake();
    currentAlgaeHeight = algaeHeight;

    switch (algaeHeight) {
      case LOW -> {
        if (needSafeAlgaeTransfer(RobotStates.FLOOR_ALGAE)) {
          return;
        }

        biscuitSubsystem.setPosition(BiscuitConstants.kFloorAlgaeSetpoint);
        elevatorSubsystem.setPosition(ElevatorConstants.kFloorAlgaeSetpoint);

        setState(RobotStates.FLOOR_ALGAE, true);
      }
      case HIGH -> {
        if (needSafeAlgaeTransfer(RobotStates.MIC_ALGAE)) {
          return;
        }

        biscuitSubsystem.setPosition(BiscuitConstants.kMicAlgaeSetpoint);
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

  public void releaseAlgae() {
    scoringTimer.stop();
    scoringTimer.reset();
    scoringTimer.start();
    isEjectingAlgae = true;
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
    currentAlgaeHeight = algaeHeight;
    if (needSafeAlgaeTransfer(RobotStates.HP_ALGAE)) {
      return;
    }

    algaeSubsystem.intake();

    biscuitSubsystem.setPosition(BiscuitConstants.kHpAlgaeSetpoint);
    elevatorSubsystem.setPosition(ElevatorConstants.kHpAlgaeSetpoint);

    setState(RobotStates.HP_ALGAE, true);
  }

  private void toProcessor() {
    currentAlgaeHeight = algaeHeight;
    if (needSafeAlgaeTransfer(RobotStates.PROCESSOR_ALGAE)) {
      return;
    }

    biscuitSubsystem.setPosition(BiscuitConstants.kProcessorSetpoint);
    elevatorSubsystem.setPosition(ElevatorConstants.kProcessorSetpoint);

    setState(RobotStates.PROCESSOR_ALGAE, true);
  }

  public void toInterrupted() {
    if (tagAlignSubsystem.getState() != TagAlignSubsystem.TagAlignStates.DONE) {
      tagAlignSubsystem.terminate();
      driveSubsystem.setIgnoreSticks(false);
    }

    biscuitSubsystem.setPosition(biscuitSubsystem.getPosition());
    elevatorSubsystem.setPosition(elevatorSubsystem.getPosition());

    setState(RobotStates.INTERRUPTED);
  }

  public void toPrepClimb() {
    if (needSafeAlgaeTransfer(RobotStates.PREP_CLIMB)) {
      return;
    }

    climbSubsystem.prepClimb();

    setState(RobotStates.PREP_CLIMB, true);
  }

  public void toClimb() {
    if (curState == RobotStates.PREP_CLIMB) {

      climbSubsystem.climb();

      setState(RobotStates.CLIMB, true);
    }
  }

  @Override
  public void periodic() {
    if (funnelSubsystem.hasCoral()) {
      coralLoc = CoralLoc.FUNNEL;
    }

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

    switch (curState) {
      case TRANSFER -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()
        // && climbSubsystem.isFinished()
        ) {
          setState(nextState);
        }
      }

      case TO_STOW -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          setState(RobotStates.STOW);
        }
      }

      case TO_STOW_SAFE -> {
        if (biscuitSubsystem.isSafeToStow()) {
          elevatorSubsystem.setPosition(ElevatorConstants.kStowSetpoint);
          setState(RobotStates.TO_STOW);
        }
      }

      case TO_STOW_SEQUENTIAL -> {
        if (biscuitSubsystem.isFinished()) {
          elevatorSubsystem.setPosition(ElevatorConstants.kStowSetpoint);
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
            || tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE
            || tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.TAG_ALIGN) {
          toReefAlign(getAlgaeOnCycle, false);
        }
      }
      case REEF_ALIGN_ALGAE -> {
        if (!isAutoPlacing
            || tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE) {
          if (algaeSubsystem.hasAlgae()) {
            switch (getAlgaeLevel()) {
              case L2 -> {
                biscuitSubsystem.setPosition(BiscuitConstants.kL2AlgaeRemovalSetpoint);
                elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeRemovalSetpoint);
              }
              case L3 -> {
                biscuitSubsystem.setPosition(BiscuitConstants.kL3AlgaeRemovalSetpoint);
                elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeRemovalSetpoint);
              }
              default -> logger.error("Invalid algae level: {}", getAlgaeLevel());
            }

            setState(RobotStates.REMOVE_ALGAE);
          }
        }
      }
      case REEF_ALIGN_CORAL -> {
        if (currentLevel != scoringLevel) {
          toReefAlign(false, false);
        }
        if (isAutoPlacing
            && tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE
            && elevatorSubsystem.isFinished()) {
          toPlaceCoral();
        }
      }
      case REMOVE_ALGAE -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          if (coralSubsystem.hasCoral()) {
            toReefAlign(false, false);
          } else {
            toStowSequential();
          }
        }
      }

      case PLACE_CORAL -> {
        if (!coralSubsystem.hasCoral()
            && scoringTimer.hasElapsed(RobotStateConstants.kCoralEjectTimer)) {
          coralLoc = CoralLoc.NONE;
          toFunnelLoad();
        } else {
          coralLoc = CoralLoc.SCORING;
        }
      }

      case FUNNEL_LOAD -> {
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
          biscuitSubsystem.setPosition(BiscuitConstants.kPrestageSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);
          funnelSubsystem.stopMotor();

          setState(RobotStates.PRESTAGE, true);
        }
      }
      case PRESTAGE -> {}
      case HP_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toProcessor();
        }
      }
      case PROCESSOR_ALGAE -> {
        if (!algaeSubsystem.hasAlgae()
            && scoringTimer.hasElapsed(RobotStateConstants.kCoralEjectTimer)
            && isEjectingAlgae) {
          isEjectingAlgae = false;
          toStowSafe();
        } else if (algaeHeight != currentAlgaeHeight) {
          toScoreAlgae();
        }
      }
      case TO_BARGE_ALGAE -> {
        if (elevatorSubsystem.isHigherThan(ElevatorConstants.kBargeHigherThan)) {
          Angle biscuitSetpoint;
          double yaw = driveSubsystem.getPoseMeters().getRotation().getDegrees();
          if (driveSubsystem.getPoseMeters().getX() <= DriveConstants.kCenterLineX) {
            biscuitSetpoint =
                yaw < 90 && yaw > -90
                    ? BiscuitConstants.kBargeSetpoint
                    : BiscuitConstants.kBargeBackwardSetpoint;
          } else {
            biscuitSetpoint =
                yaw < -90 || yaw > 90
                    ? BiscuitConstants.kBargeSetpoint
                    : BiscuitConstants.kBargeBackwardSetpoint;
          }
          biscuitSubsystem.setPosition(biscuitSetpoint);
          curState = RobotStates.BARGE_ALGAE;
        }
      }
      case BARGE_ALGAE -> {
        if (!algaeSubsystem.hasAlgae()
            && scoringTimer.hasElapsed(RobotStateConstants.kCoralEjectTimer)
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
      case PREP_CLIMB -> {}
      case CLIMB -> {}
      case INTERRUPTED -> {}
      case IDLE -> {}
      default -> logger.error("Unhandled state: {}", curState);
    }
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
    IDLE
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
