package frc.robot.subsystems.robotState;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.Measure;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.robotState.SetAutoPlacingCommand;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.constants.TagServoingConstants;
import frc.robot.standards.StateMachineSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;
import frc.robot.subsystems.battMon.BattMonSubsystem;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;
import frc.robot.subsystems.climb.ClimbAlignSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDStates;
import frc.robot.subsystems.tagAlign.BargeAlignSubsystem;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem;
import frc.robot.subsystems.tagAlign.BargeAlignSubsystem.BargeAlignStates;
import frc.robot.subsystems.tagAlign.TagAlignSubsystem.TagAlignStates;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.robotState.NewRobotStates.*;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotStates;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;

public class NewRobotStateSubsystem extends StateMachineSubsystem<NewRobotStates> {
    //Subsystems & Hardware
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
    private BargeAlignSubsystem bargeAlignSubsystem;
    private CANBus canBus;

    //Local Variables
    private Alliance allianceColor = Alliance.Blue;
    private NewRobotStates nextState;
    private NewRobotStates futureState;
    private NewScoringLevel scoringLevel = NewScoringLevel.L4;
    private NewScoringLevel curLevel = NewScoringLevel.L4;
    private NewScoringLevel autoAlgaeLevel = NewScoringLevel.L2;
    private NewScoreSide scoreSide = NewScoreSide.LEFT;
    private NewAlgaeHeight algaeHeight = NewAlgaeHeight.LOW;
    private NewAlgaeHeight curAlgaeHeight = NewAlgaeHeight.LOW;
    private NewCoralLoc coralLoc = NewCoralLoc.NONE;
    private Pose2d processorReleasePose;
    private NewRobotStates interruptedState = NewRobotStates.IDLE;
    private Angle nextBiscuitSetpoint;
    private Timer scoringTimer = new Timer();

    //Control Booleans
    private boolean isAutoPlacing = false;
    private boolean getAlgaeOnCycle = false;
    private boolean isCurrentLimiting = false;
    private boolean isAuto = false;
    private boolean isEjectingAlgae = false;
    private boolean isBargeSafe = true;
    private boolean isAutoReadyForEject = false;
    private boolean isPrestagingForAlgae = false;
    private boolean hasElevatorAutoCoralPrestaged = false;
    private boolean reefCoralStuckFixable = false;
    private boolean stuckAndMisaligned = false;

    public NewRobotStateSubsystem(
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
        VisionSubsystem visionSubsystem,
        BargeAlignSubsystem bargeAlignSubsystem) {
            super(NewRobotStates.STARTUP);

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
            this.bargeAlignSubsystem = bargeAlignSubsystem;
            this.canBus = new CANBus("CAN FD 25-1");

            ledSubsystem.setState(LEDStates.NORMAL);
        }

    public boolean hasCoral() {
        return false;
    }

    public boolean hasAlgae() {
        return false;
    }

    private boolean justAlgae() {
        boolean blueSide = driveSubsystem.getPoseMeters().getX() < DriveConstants.kFieldMaxX/2;

        return !hasCoral() || (blueSide && allianceColor == Alliance.Red) || (!blueSide && allianceColor == Alliance.Blue);
    }

    private void setBiscuitTransfer(Angle setpoint, boolean overrideThreshold) {
        if(overrideThreshold || elevatorSubsystem.getPosition().gt(ElevatorConstants.kBiscuitSafeThreshold)) biscuitSubsystem.setPosition(setpoint, hasAlgae());
        else nextBiscuitSetpoint = setpoint;
    }

    private ScoringLevel getOldLevel(NewScoringLevel level) {
        switch(level) {
            case L1: return ScoringLevel.L1;
            case L2: return ScoringLevel.L2;
            case L3: return ScoringLevel.L3;
            case L4: return ScoringLevel.L4;
            default: return ScoringLevel.L1;
        }
    }

    private NewScoringLevel getAlgaLevel() {
        return isAuto ? autoAlgaeLevel : (tagAlignSubsystem.computeHexant() % 2) == 0 ? NewScoringLevel.L3 : NewScoringLevel.L2;
    }

    private void prepForStow() {
        visionSubsystem.setYawUpdateCamera(-1);
        visionSubsystem.setIgnoreRearCams(false);
        visionSubsystem.setUsingLeftCam(true);
        visionSubsystem.setUsingRightCam(true);
        biscuitSubsystem.setIsRemovingAlgae(false);
        driveSubsystem.removeDriveMultiplier();
        driveSubsystem.setIgnoreSticks(false);

        if(tagAlignSubsystem.getState() != TagAlignStates.DONE) tagAlignSubsystem.terminate();
        if(bargeAlignSubsystem.getState() != BargeAlignStates.FINISHED) bargeAlignSubsystem.terminate();

        if(algaeSubsystem.hasCoral()) algaeSubsystem.holdCoral();
        else algaeSubsystem.holdAlgae();
    }

    private void setCurState(NewRobotStates desiredState, boolean throughTransfer) {
        if(curState != desiredState) {
            if(throughTransfer) {
                nextState = desiredState;
                curState = NewRobotStates.TRANSFER;
            } else {
                curState = nextState = desiredState;
            }
        }
    }

    private void setCurState(NewRobotStates desiredState) {
        setCurState(desiredState, false);
    }

    private NewRobotStates getNextReefAlignState(boolean getAlgae, boolean autoDrive){
        boolean wantAlgae = (getAlgae || !hasCoral() && scoringLevel != NewScoringLevel.L1);
        boolean ignoreCoralScoring = isAutoPlacing && getAlgaeOnCycle && scoreSide == NewScoreSide.RIGHT;
        boolean algaeSafe = tagAlignSubsystem.getCurRadius() > TagServoingConstants.kAlgaeStopXDriveRadius || tagAlignSubsystem.getState() != TagAlignStates.DRIVE;
        if((hasAlgae() || !wantAlgae) && (!hasCoral() || ignoreCoralScoring)) return NewRobotStates.TO_STOW;
        else if(wantAlgae && algaeSafe && (scoreSide == NewScoreSide.LEFT || !isAutoPlacing || ignoreCoralScoring) && !algaeSubsystem.hasAlgae()) return NewRobotStates.REEF_ALIGN_ALGAE;
        else if(autoDrive) return NewRobotStates.REEF_ALIGN_CORAL;
        else if(!autoDrive) {
            if(!isAutoPlacing || (tagAlignSubsystem.yErrorSmall() && tagAlignSubsystem.getCurRadius() < RobotStateConstants.kElevatorWaitRadius)) return NewRobotStates.REEF_ALIGN_CORAL;
            else return NewRobotStates.REEF_ALIGN;
        }
        else return NewRobotStates.REEF_ALIGN;
    }

    private void prepForAutoReefAlign(boolean wantAlgae) {
        boolean getAlgae = wantAlgae || !hasCoral() && scoringLevel != NewScoringLevel.L1;
        visionSubsystem.setIgnoreRearCams(true);
        reefCoralStuckFixable = false;

        tagAlignSubsystem.setJustAlgae(justAlgae());
        tagAlignSubsystem.start(allianceColor, getOldLevel(scoringLevel), scoreSide == NewScoreSide.LEFT || getAlgae, getAlgae);
        ledSubsystem.setAutoPlacing(true);
        stuckAndMisaligned = false;

        if(getAlgae) {
            algaeSubsystem.intakeAlgae();
            switch(getAlgaLevel()) {
                case L2 -> {
                    setBiscuitTransfer(RobotConstants.kL2AlgaeSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
                }
                case L3 -> {
                    setBiscuitTransfer(RobotConstants.kL3AlgaeSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeSetpoint);
                }
                default -> System.out.println("error");
            }
        }
    }

    private void changeScoringLevel() {
        if(!isAutoPlacing || (tagAlignSubsystem.yErrorSmall() && tagAlignSubsystem.getCurRadius() < RobotStateConstants.kElevatorWaitRadius)) {
            curLevel = scoringLevel;

            int hexant = tagAlignSubsystem.computeHexant();
            var correctElevatorOffsetMatrix = allianceColor == Alliance.Blue ? RobotStateConstants.kBlueCoralElevatorOffset : RobotStateConstants.kRedCoralElevatorOffset;
            Angle elevatorOffset = correctElevatorOffsetMatrix[hexant][scoringLevel.ordinal()][scoreSide.ordinal()];
            switch(scoringLevel) {
                case L1 -> {
                    setBiscuitTransfer(RobotConstants.kL1CoralSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL1CoralSetpoint.plus(elevatorOffset));
                }
                case L2 -> {
                    setBiscuitTransfer(RobotConstants.kL2CoralSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL2CoralSetpoint.plus(elevatorOffset));
                }
                case L3 -> {
                    setBiscuitTransfer(RobotConstants.kL3CoralSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL3CoralSetpoint.plus(elevatorOffset));
                }
                case L4 -> {
                    setBiscuitTransfer(RobotConstants.kL4CoralSetpoint, true);
                    elevatorSubsystem.setPosition(ElevatorConstants.kL4CoralSetpoint.plus(elevatorOffset));
                }
            }
            hasElevatorAutoCoralPrestaged = true;
        }
        hasElevatorAutoCoralPrestaged = false;
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.recordOutput("RobotState/state", curState);
        Logger.recordOutput("RobotState/futureState", futureState);
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
        Logger.recordOutput("RobotState/stateDuration", timeInState);
    }

    @Override
    protected NewRobotStates shouldTransitionStates() {
        switch(curState) {
            case TRANSFER -> {
                if(biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished() || ((nextState == NewRobotStates.PREP_CLIMB || nextState == NewRobotStates.CLIMB) && elevatorSubsystem.getPosition().in(Rotations) < RobotStateConstants.kElevatorClimbMax)) return nextState;
                else return curState;
            }
            case STOW -> {
                if(futureState != null) return futureState;
                else if(!hasCoral()) return NewRobotStates.FUNNEL_LOAD;
                else return curState;
            }
            case TO_STOW -> {
                if(biscuitSubsystem.isFinished() && elevatorSubsystem.isFinished()) return NewRobotStates.STOW;
                else return curState;
            }
            case TO_STOW_SAFE -> {
                if(biscuitSubsystem.isSafeToStow()) return NewRobotStates.TO_STOW;
                else return curState;
            }
            case TO_STOW_SEQUENTIAL -> {
                if(biscuitSubsystem.isFinished()) return NewRobotStates.TO_STOW;
                else return curState;
            }
            case FUNNEL_LOAD -> {
                if(scoringLevel == NewScoringLevel.L1) return NewRobotStates.TO_ALGAE_CORAL_LOAD;
                else if(funnelSubsystem.hasCoral()) {
                    coralLoc = NewCoralLoc.TRANSFER;
                    return NewRobotStates.LOADING_CORAL;
                }
                else return curState;
            }
            case LOADING_CORAL -> {
                if(coralSubsystem.hasCoral()) return NewRobotStates.PRESTAGE;
                else return curState;
            }
            case TO_ALGAE_CORAL_LOAD -> {
                if(scoringLevel != NewScoringLevel.L1) return NewRobotStates.FUNNEL_LOAD;
                else if(elevatorSubsystem.isFinished()) return NewRobotStates.ALGAE_CORAL_LOAD;
                else return curState;
            }
            case ALGAE_CORAL_LOAD -> {
                if(scoringLevel != NewScoringLevel.L1) return NewRobotStates.FUNNEL_LOAD;
                else if(algaeSubsystem.hasCoral()) {
                    coralLoc = NewCoralLoc.ALGAE;
                    return NewRobotStates.PRESTAGE;
                }
                else return curState;
            }
            case PRESTAGE -> {
                if(getAlgaeOnCycle != isPrestagingForAlgae) {
                    nextState = NewRobotStates.PRESTAGE;

                    setBiscuitTransfer(RobotConstants.kPrestageSetpoint, false);
                    isPrestagingForAlgae = getAlgaeOnCycle;
                    if(getAlgaeOnCycle) elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
                    else elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);
                    funnelSubsystem.stopMotor();

                    return NewRobotStates.TRANSFER;
                } else return curState;
            }
            case REEF_ALIGN -> {
                if(!isAutoPlacing || tagAlignSubsystem.getState() != TagAlignStates.DRIVE) return getNextReefAlignState(getAlgaeOnCycle, false);
                else return curState;
            }
            case REEF_ALIGN_CORAL -> {
                if(isAutoPlacing && (tagAlignSubsystem.getState() == TagAlignStates.DONE) && elevatorSubsystem.isFinished()) return NewRobotStates.PLACE_CORAL;
                else return curState;
            }
            case REEF_ALIGN_ALGAE -> {
                if(algaeSubsystem.hasAlgaeSuperCycle()) return NewRobotStates.REMOVE_ALGAE;
                else return curState;
            }
            case PLACE_CORAL -> {
                if(!hasCoral() && scoringTimer.hasElapsed(RobotStateConstants.kCoralEjectTimer)) {
                    coralLoc = NewCoralLoc.NONE;
                    visionSubsystem.setYawUpdateCamera(-1);
                    ledSubsystem.setAutoPlacing(false);
                    driveSubsystem.setIgnoreSticks(false);
                    if(scoringLevel == NewScoringLevel.L1 && tagAlignSubsystem.getCurRadius() < RobotStateConstants.kL1CoralStowRadius) return curState;
                    else return NewRobotStates.TO_STOW_SAFE;
                }
                else return curState;
            }
            case REMOVE_ALGAE -> {
                if(elevatorSubsystem.isFinished() && biscuitSubsystem.getPosition().in(Rotations) <= RobotStateConstants.kBiscuitSuperCycleSafeThres) {
                    biscuitSubsystem.setIsRemovingAlgae(false);
                    boolean blueSide = driveSubsystem.getPoseMeters().getX() < DriveConstants.kFieldMaxX/2;
                    if(coralSubsystem.hasCoral() && !(isAutoPlacing && getAlgaeOnCycle && scoreSide == NewScoreSide.RIGHT) && !(blueSide && allianceColor == Alliance.Red || !blueSide && allianceColor == Alliance.Blue)) {
                        return getNextReefAlignState(false, false);
                    } else {
                        ledSubsystem.setAutoPlacing(false);
                        return NewRobotStates.TO_STOW_SEQUENTIAL;
                    }
                } else return curState;
            }
            default -> {
                return curState;
            }
        }
    }

    @Override
    protected void transitionStates(NewRobotStates newState) {
        super.transitionStates(newState);
        if(curState == newState) return;
        switch(newState) {
            case TRANSFER -> {
                curState = NewRobotStates.TRANSFER;
            }
            case TO_STOW -> {
                prepForStow();

                if(biscuitSubsystem.isSafeToStow()) { 
                    setBiscuitTransfer(RobotConstants.kStowSetpoint, true);
                    elevatorSubsystem.setPosition(RobotConstants.kElevatorStowSetpoint);
                    setCurState(NewRobotStates.TO_STOW);
                } else transitionStates(NewRobotStates.TO_STOW_SAFE);
            }
            case TO_STOW_SAFE -> {
                prepForStow();
                setBiscuitTransfer(RobotConstants.kStowSetpoint, true);
                setCurState(NewRobotStates.TO_STOW_SAFE);
            }
            case TO_STOW_SEQUENTIAL -> {
                prepForStow();
                setBiscuitTransfer(RobotConstants.kStowSetpoint, true);
                setCurState(NewRobotStates.TO_STOW_SEQUENTIAL);
            }
            case FUNNEL_LOAD -> {
                setBiscuitTransfer(RobotConstants.kFunnelSetpoint, true);
                algaeSubsystem.holdAlgae();
                elevatorSubsystem.setPosition(RobotConstants.kElevatorFunnelSetpoint);
                coralSubsystem.intake();
                setCurState(NewRobotStates.FUNNEL_LOAD, true);
            }
            case TO_ALGAE_CORAL_LOAD -> {
                elevatorSubsystem.setPosition(RobotConstants.kElevatorL1LoadSetpoint);
                funnelSubsystem.reverse();
                coralSubsystem.stop();
                setCurState(NewRobotStates.TO_ALGAE_CORAL_LOAD, false);
            }
            case ALGAE_CORAL_LOAD -> {
                setBiscuitTransfer(RobotConstants.kL1CoralLoadSetpoint, true);
                setCurState(NewRobotStates.ALGAE_CORAL_LOAD, true);
            }
            case PRESTAGE -> {
                if(isAuto) {
                    coralLoc = NewCoralLoc.CORAL;
                    setBiscuitTransfer(RobotConstants.kPrestageSetpoint, false);
                    elevatorSubsystem.setPosition(ElevatorConstants.kAutoPrestageSetpoint);
                    funnelSubsystem.stopMotor();
                    setCurState(NewRobotStates.PRESTAGE, true);
                } else {
                    setBiscuitTransfer(RobotConstants.kPrestageSetpoint, false);
                    isPrestagingForAlgae = getAlgaeOnCycle;
                    if(getAlgaeOnCycle) elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeSetpoint);
                    else elevatorSubsystem.setPosition(ElevatorConstants.kPrestageSetpoint);
                    funnelSubsystem.stopMotor();
                    setCurState(NewRobotStates.PRESTAGE, true);
                }
            }
            case REEF_ALIGN -> {
                if(isAutoPlacing) prepForAutoReefAlign(getAlgaeOnCycle);
                changeScoringLevel();
                setCurState(NewRobotStates.REEF_ALIGN);
            }
            case PLACE_CORAL -> {
                isAutoReadyForEject = false;
                reefCoralStuckFixable = false;
                if(isAutoPlacing) visionSubsystem.setYawUpdateCamera(scoreSide == NewScoreSide.LEFT ? 2: 0);

                if(algaeSubsystem.hasCoral() && scoringLevel == NewScoringLevel.L1) algaeSubsystem.scoreCoral();
                else coralSubsystem.eject(getOldLevel(scoringLevel));

                funnelSubsystem.clearCoral();
                scoringTimer.stop();
                scoringTimer.reset();
                scoringTimer.start();
                setCurState(NewRobotStates.PLACE_CORAL);
            }
            case REMOVE_ALGAE -> {
                switch(getAlgaLevel()) {
                    case L2 -> {
                        biscuitSubsystem.setPosition(RobotConstants.kL2AlgaeRemovalSetpoint, true);
                        elevatorSubsystem.setPosition(ElevatorConstants.kL2AlgaeRemovalSetpoint);
                    }
                    case L3 -> {
                        biscuitSubsystem.setPosition(RobotConstants.kL3AlgaeRemovalSetpoint, true);
                        elevatorSubsystem.setPosition(ElevatorConstants.kL3AlgaeRemovalSetpoint);
                    }
                    default -> System.out.println("error");
                }
                setCurState(NewRobotStates.REMOVE_ALGAE);
            }
            default -> {
                return;
            }
        }
    }

    @Override
    protected void executeState() {
        switch(curState) {
            case TRANSFER -> {
                if(elevatorSubsystem.getPosition().gt(ElevatorConstants.kBiscuitSafeThreshold)) biscuitSubsystem.setPosition(nextBiscuitSetpoint, hasAlgae());
            }
            case FUNNEL_LOAD -> {
                if(elevatorSubsystem.isFinished()) funnelSubsystem.startMotor();
            }
            case ALGAE_CORAL_LOAD -> {
                double curX = driveSubsystem.getPoseMeters().getX();
                if(allianceColor == Alliance.Blue && curX < RobotStateConstants.kL1FunnelLoadX || allianceColor == Alliance.Red && curX > (DriveConstants.kFieldMaxX - RobotStateConstants.kL1FunnelLoadX)) {
                    if(algaeSubsystem.getState() != AlgaeStates.CORAL_INTAKE && algaeSubsystem.getState() != AlgaeStates.HAS_CORAL) algaeSubsystem.intakeCoral();
                } else if(algaeSubsystem.getState() == AlgaeStates.CORAL_INTAKE) algaeSubsystem.holdAlgae();
            }
            case REEF_ALIGN -> {
                if(scoringLevel != curLevel) changeScoringLevel();
            }
            case REEF_ALIGN_CORAL -> {
                if(scoringLevel != curLevel) changeScoringLevel();
                else if(!hasElevatorAutoCoralPrestaged && tagAlignSubsystem.yErrorSmall()) changeScoringLevel();
            }
            case PLACE_CORAL -> {
                if(hasCoral()) coralLoc = NewCoralLoc.SCORING;
            }
            default -> {
                return;
            }
        }
    }
    

    


    @Override
    public Set<Measure> getMeasures() {
        return Set.of();
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        super.registerWith(telemetryService);
    }
}
