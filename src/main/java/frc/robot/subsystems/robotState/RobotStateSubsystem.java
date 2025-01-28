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
  private LEDSubsystem ledSubsystem;
  private TagAlignSubsystem tagAlignSubsystem;
  private VisionSubsystem visionSubsystem;

  private RobotStates curState;
  private RobotStates nextState;
  private ScoringLevel scoringLevel;
  private ScoreSide scoreSide;
  private AlgaeHeight algaeHeight;
  private CoralLoc coralLoc;

  private boolean isAutoPlacing = true;
  private boolean getAlgaeOnCycle = true;
  private boolean isCurrentLimiting = false;
  private boolean hasCoral = true;
  private boolean hasAlgae = false;
  private boolean isAuto = true;

  private Pose2d algaeRemovalPose;
  private ScoringLevel algaeLevel;

  public RobotStateSubsystem(
      AlgaeSubsystem algaeSubsystem,
      BattMonSubsystem battMonSubsystem,
      BiscuitSubsystem biscuitSubsystem,
      ClimbSubsystem climbSubsystem,
      CoralSubsystem coralSubsystem,
      DriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
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
    this.ledSubsystem = ledSubsystem;
    this.tagAlignSubsystem = tagAlignSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  public RobotStates getState() {
    return curState;
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

  private void setState(RobotStates robotState, boolean transfer) {
    if (curState != robotState) {
      if (transfer) {
        nextState = robotState;
        logger.info("TRANSFER ({} -> {})", curState, nextState);
      } else {
        logger.info("{} -> {}", this.curState, robotState);
      }
      curState = robotState;
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

  public void setCoralLoc(CoralLoc coralLoc) {
    this.coralLoc = coralLoc;
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

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
  }

  public void toStow() {
    algaeSubsystem.hold();
    biscuitSubsystem.setPosition(BiscuitConstants.kStowSetpoint);
    coralSubsystem.hold();
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

  public void toFunnelLoad() {
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
      algaeSubsystem.intake();

      switch (getAlgaeLevel()) {
        case L1 -> {}
        case L2 -> {
          biscuitSubsystem.setPosition(BiscuitConstants.kL2AlgaeSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
        }
        case L3 -> {
          biscuitSubsystem.setPosition(BiscuitConstants.kL3AlgaeSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeSetpoint);
        }
        case L4 -> {}
      }
    } else {
      if (!coralSubsystem.hasCoral()) {
        toStow();
        return;
      } else {
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
      tagAlignSubsystem.setup();
      tagAlignSubsystem.start();
    }

    setState(RobotStates.REEF_ALIGN);
  }

  public void toRemoveAlgae() {
    algaeRemovalPose = driveSubsystem.getPoseMeters();

    switch (getAlgaeLevel()) {
      case L2 -> {
        biscuitSubsystem.setPosition(BiscuitConstants.kL2AlgaeRemovalSetpoint);
        elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeRemovalSetpoint);

        switch (scoringLevel) {
          case L1, L2 -> {
            driveSubsystem.move(DriveConstants.kAlgaeRemovalSpeed, 0, 0, false);
          }
        }
      }
      case L3 -> {
        biscuitSubsystem.setPosition(BiscuitConstants.kL3AlgaeRemovalSetpoint);
        elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeRemovalSetpoint);
      }
    }

    setState(RobotStates.REMOVE_ALGAE);
  }

  public void toPlaceCoral() {
    coralSubsystem.place();

    setState(RobotStates.PLACE_CORAL);
  }

  @Override
  public void periodic() {
    switch (curState) {
      case TRANSFER -> {
        if (biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) {
          setState(nextState);
        }
      }

      case TO_STOW -> {
        if (algaeSubsystem.isFinished()
            && biscuitSubsystem.isFinished()
            && coralSubsystem.isFinished()
            && elevatorSubsystem.isFinished()) {
          setState(RobotStates.STOW);
        }
      }
      case STOW -> {
        if (!coralSubsystem.hasCoral()) {
          toFunnelLoad();
        }
      }
      case PREP_CORAL -> {}
      case REEF_ALIGN -> {
        if (tagAlignSubsystem.getState() == TagAlignSubsystem.TagAlignStates.DONE) {
          if (getAlgaeOnCycle && algaeSubsystem.hasAlgae()) {
            toRemoveAlgae();
          } else if (!getAlgaeOnCycle) {
            toPlaceCoral();
          }
        }
      }
      case REMOVE_ALGAE -> {
        if (getAlgaeLevel() == ScoringLevel.L2) {
          switch (scoringLevel) {
            case L1, L2 -> {
              if (driveSubsystem.getPoseMeters().minus(algaeRemovalPose).getTranslation().getNorm()
                  > RobotStateConstants.kAlgaeRetreatDistance) {
                driveSubsystem.move(0, 0, 0, false);
                toReefAlign(false, true);
              }
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

      case PLACE_CORAL -> {
        if (!coralSubsystem.hasCoral()) {
          toFunnelLoad();
        }
      }

      case FUNNEL_LOAD -> {
        /*if ( FIXME: funnel beam break? )*/ {
          setState(RobotStates.LOADING_CORAL);
        }
      }
      case LOADING_CORAL -> {
        if (coralSubsystem.hasCoral()) {
          biscuitSubsystem.setPosition(BiscuitConstants.kPrestageSetpoint);
          elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);

          setState(RobotStates.PRESTAGE, true);
        }
      }
      case PRESTAGE -> {}
      case HP_ALGAE -> {}
      case PROCESSOR_ALGAE -> {}
      case BARGE_ALGAE -> {}
      case MIC_ALGAE -> {}
      case FLOOR_ALGAE -> {}
      case PRE_CLIMB -> {}
      case CLIMB -> {}
      case INTERRUPTED -> {}
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
    REEF_ALIGN,
    REMOVE_ALGAE,
    CORAL_REALIGN,
    PLACE_CORAL,
    FUNNEL_LOAD,
    LOADING_CORAL,
    PRESTAGE,
    HP_ALGAE,
    PROCESSOR_ALGAE,
    BARGE_ALGAE,
    MIC_ALGAE,
    FLOOR_ALGAE,
    PRE_CLIMB,
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
    High
  }

  public enum CoralLoc {
    FUNNEL,
    TRANSFER,
    CORAL,
    SCORING,
    NONE
  }
}
