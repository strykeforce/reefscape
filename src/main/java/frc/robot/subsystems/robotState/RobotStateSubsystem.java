package frc.robot.subsystems.robotState;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

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

public class RobotStateSubsystem extends MeasurableSubsystem {
    private org.slf4j.Logger logger = LoggerFactory.getLogger(this.getClass());

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

    public RobotStateSubsystem(AlgaeSubsystem algaeSubsystem, BattMonSubsystem battMonSubsystem,
            BiscuitSubsystem biscuitSubsystem, ClimbSubsystem climbSubsystem, CoralSubsystem coralSubsystem,
            DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem,
            TagAlignSubsystem tagAlignSubsystem, VisionSubsystem visionSubsystem) {
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

    private void setState(RobotStates robotState) {
        if (this.curState != robotState) {
            logger.info("{} -> {}", this.curState, robotState);
            this.curState = robotState;
        }
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

    @Override
    public void periodic() {
        switch (curState) {
            case TO_STOW -> {
            }
            case STOW -> {
            }
            case PREP_CORAL -> {
            }
            case REEF_ALIGN -> {
            }
            case REMOVE_ALGAE -> {
            }
            case CORAL_REALIGN -> {
            }
            case PLACE_CORAL -> {
            }
            case FUNNEL_LOAD -> {
            }
            case LOADING_CORAL -> {
            }
            case PRESTAGE -> {
            }
            case HP_ALGAE -> {
            }
            case PROCESSOR_ALGAE -> {
            }
            case BARGE_ALGAE -> {
            }
            case MIC_ALGAE -> {
            }
            case FLOOR_ALGAE -> {
            }
            case PRE_CLIMB -> {
            }
            case CLIMB -> {
            }
            case INTERRUPTED -> {
            }
        }
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(
                new Measure("state", () -> curState.ordinal()));
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
    }

    public enum ScoringLevel {
        L1, L2, L3, L4
    }

    public enum ScoreSide {
        LEFT, RIGHT
    }

    public enum AlgaeHeight {
        LOW, High
    }

    public enum CoralLoc {
        FUNNEL, TRANSFER, CORAL, SCORING, NONE
    }
}
