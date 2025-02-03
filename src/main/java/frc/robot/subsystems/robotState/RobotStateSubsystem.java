package frc.robot.subsystems.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
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

  private RobotStates curState;
  private RobotStates nextState;
  private RobotStates futureState;

  private ScoringLevel scoringLevel;
  private ScoreSide scoreSide;
  private AlgaeHeight algaeHeight;
  private CoralLoc coralLoc;

  private boolean isAutoPlacing = true;
  private boolean getAlgaeOnCycle = true;
  private boolean isCurrentLimiting = false;
  private boolean isAuto = true;

  private Pose2d algaeRemovalPose;

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

  public void setAllianceColor(Alliance alliance) {
    allianceColor = alliance;
    logger.info("Change color to: {}", allianceColor);
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public ScoringLevel getAlgaeLevel() {
    return tagAlignSubsystem.getHexant() % 2 == 0 ? ScoringLevel.L3 : ScoringLevel.L2;
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
      }
      curState = nextState = robotState;
    }
  }

  private void setState(RobotStates robotState) {
    setState(robotState, false);
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

  public void setAutoPlacing(boolean isAutoPlacing) {
    this.isAutoPlacing = isAutoPlacing;
  }

  public void setGetAlgaeOnCycle(boolean getAlgaeOnCycle) {
    this.getAlgaeOnCycle = getAlgaeOnCycle;
  }

  public void setCurrentLimiting(boolean isCurrentLimiting) {
    this.isCurrentLimiting = isCurrentLimiting;
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
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
              toStow();
              return true;
            }
            default -> {}
          }
        }
        default -> {}
      }

      switch (curState) {
        case REEF_ALIGN_ALGAE,
            REEF_ALIGN_CORAL,
            REMOVE_ALGAE,
            SAFE_REMOVE_ALGAE_ABOVE,
            SAFE_REMOVE_ALGAE_ROTATE,
            PLACE_CORAL,
            INTERRUPTED -> {
          switch (nextState) {
              // These are all the states that could possibly be entered that are also
              // dangerous
              // Each futureState must be handled in STOW
            case FLOOR_ALGAE, MIC_ALGAE, PROCESSOR_ALGAE, BARGE_ALGAE, HP_ALGAE, PREP_CLIMB -> {
              futureState = nextState;
              toStow();
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
    biscuitSubsystem.setPosition(BiscuitConstants.kStowSetpoint);
    elevatorSubsystem.setPosition(ElevatorConstants.kStowSetpoint);

    setState(RobotStates.TO_STOW);
  }

  public void toPrepCoral() {
    if (isAuto) {
      toReefAlign(false, false);
    } else {
      toReefAlign(getAlgaeOnCycle, false);
    }
  }

  private void toFunnelLoad() {
    biscuitSubsystem.setPosition(BiscuitConstants.kFunnelSetpoint);
    coralSubsystem.intake();
    elevatorSubsystem.setPosition(ElevatorConstants.kFunnelSetpoint);

    setState(RobotStates.FUNNEL_LOAD, true);
  }

  public void toReefAlign() {
    toReefAlign(getAlgaeOnCycle, true);
  }

  private void toReefAlign(boolean getAlgae, boolean drive) {
    if (getAlgae) {
      if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_ALGAE)) {
        return;
      }
      setState(RobotStates.REEF_ALIGN_ALGAE);

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
    } else {
      if (!coralSubsystem.hasCoral()) {
        toStow();
        return;
      } else {
        if (needSafeAlgaeTransfer(RobotStates.REEF_ALIGN_CORAL)) {
          return;
        }
        setState(RobotStates.REEF_ALIGN_CORAL);

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
      }
    }

    if (drive) {
      tagAlignSubsystem.setup(allianceColor, scoreSide);
      tagAlignSubsystem.start();
    }
  }

  public void toPlaceCoral() {
    coralSubsystem.place();

    setState(RobotStates.PLACE_CORAL);
  }

  public void toAlgaeFloorPickup() {
    algaeSubsystem.intake();

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
    switch (algaeHeight) {
      case LOW -> {
        toProcessor();
      }
      case HIGH -> {
        if (needSafeAlgaeTransfer(RobotStates.BARGE_ALGAE)) {
          return;
        }

        double poseX = driveSubsystem.getPoseMeters().getX();

        if (poseX > RobotStateConstants.kRedBargeSafeX
            || poseX < RobotStateConstants.kBlueBargeSafeX) {
          biscuitSubsystem.setPosition(BiscuitConstants.kBargeSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kBargeSetpoint);

          setState(RobotStates.BARGE_ALGAE, true);
        }
      }
    }
  }

  public void releaseAlgae() {
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
    if (needSafeAlgaeTransfer(RobotStates.HP_ALGAE)) {
      return;
    }

    algaeSubsystem.intake();

    biscuitSubsystem.setPosition(BiscuitConstants.kHpAlgaeSetpoint);
    elevatorSubsystem.setPosition(ElevatorConstants.kHpAlgaeSetpoint);

    setState(RobotStates.HP_ALGAE, true);
  }

  private void toProcessor() {
    if (needSafeAlgaeTransfer(RobotStates.PROCESSOR_ALGAE)) {
      return;
    }

    biscuitSubsystem.setPosition(BiscuitConstants.kProcessorSetpoint);
    elevatorSubsystem.setPosition(ElevatorConstants.kProcessorSetpoint);

    setState(RobotStates.PROCESSOR_ALGAE, true);
  }

  public void toInterrupted() {
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

    switch (curState) {
      case TRANSFER -> {
        if (biscuitSubsystem.isFinished()
            && elevatorSubsystem.isFinished()
            && climbSubsystem.isFinished()) {
          setState(nextState);
        }
      }

      case TO_STOW -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          setState(RobotStates.STOW);
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
        // case PREP_CORAL -> {
        //   if (algaeSubsystem.hasAlgae()) {
        //     toRemoveAlgae();
        //   } else if () {
        //     toPlaceCoral();
        //   }
        // }
      case REEF_ALIGN_ALGAE -> {
        if (!isAutoPlacing
            || tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE) {
          if (algaeSubsystem.hasAlgae()) {
            algaeRemovalPose = driveSubsystem.getPoseMeters();

            switch (getAlgaeLevel()) {
              case L2 -> {
                biscuitSubsystem.setPosition(BiscuitConstants.kL2AlgaeRemovalSetpoint);
                elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeRemovalSetpoint);

                // FIXME: if driving to remove algae
                switch (scoringLevel) {
                  case L1, L2 -> {
                    driveSubsystem.move(DriveConstants.kAlgaeRemovalSpeed, 0, 0, false);
                  }
                  default -> {}
                }
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
        if (tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE) {
          toPlaceCoral();
        }
      }
      case REMOVE_ALGAE -> {
        if (getAlgaeLevel() == ScoringLevel.L2) {
          switch (scoringLevel) {
            case L1, L2 -> {
              // FIXME: if driving to remove algae
              if (driveSubsystem.getPoseMeters().minus(algaeRemovalPose).getTranslation().getNorm()
                  > RobotStateConstants.kAlgaeRetreatDistance) {
                driveSubsystem.move(0, 0, 0, false);
                toReefAlign(false, true);
              }
              // FIXME: if not driving to remove algae
              biscuitSubsystem.setPosition(BiscuitConstants.kSafeAlgaeRemovalSetpoint);
              elevatorSubsystem.setPosition(ElevatorConstants.kSafeAlgaeRemovalSetpoint);
              setState(RobotStates.SAFE_REMOVE_ALGAE_ABOVE, true);
            }
            case L3, L4 -> {
              if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
                toReefAlign(false, false);
              }
            }
          }
        } else if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          toReefAlign(false, false);
        }
      }

      case SAFE_REMOVE_ALGAE_ABOVE -> {
        biscuitSubsystem.setPosition(BiscuitConstants.kSafeAlgaeRemovalRotateSetpoint);
        elevatorSubsystem.setPosition(ElevatorConstants.kSafeAlgaeRemovalRotateSetpoint);
        setState(RobotStates.SAFE_REMOVE_ALGAE_ROTATE, true);
      }

      case SAFE_REMOVE_ALGAE_ROTATE -> {
        toReefAlign(false, false);
      }

      case PLACE_CORAL -> {
        if (!coralSubsystem.hasCoral()) {
          coralLoc = CoralLoc.NONE;
          toFunnelLoad();
        }
      }

      case FUNNEL_LOAD -> {
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
        if (!algaeSubsystem.hasAlgae()) {
          toStow();
        }
      }
      case BARGE_ALGAE -> {
        if (!algaeSubsystem.hasAlgae()) {
          toStow();
        }
      }
      case MIC_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toStow();
        }
      }
      case FLOOR_ALGAE -> {
        if (algaeSubsystem.hasAlgae()) {
          toStow();
        }
      }
      case PREP_CLIMB -> {}
      case CLIMB -> {}
      case INTERRUPTED -> {}
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
    PREP_CORAL,
    REEF_ALIGN_ALGAE,
    REEF_ALIGN_CORAL,
    REMOVE_ALGAE,
    SAFE_REMOVE_ALGAE_ABOVE,
    SAFE_REMOVE_ALGAE_ROTATE,
    // CORAL_REALIGN,
    PLACE_CORAL,
    FUNNEL_LOAD,
    LOADING_CORAL,
    PRESTAGE,
    HP_ALGAE,
    PROCESSOR_ALGAE,
    BARGE_ALGAE,
    MIC_ALGAE,
    FLOOR_ALGAE,
    PREP_CLIMB,
    CLIMB,
    INTERRUPTED,
    TRANSFER
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
