package frc.robot.subsystems.algae;

import frc.robot.constants.AlgaeConstants;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AlgaeSubsystem extends MeasurableSubsystem {
  private org.slf4j.Logger logger = LoggerFactory.getLogger(AlgaeSubsystem.class);

  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  private double desiredSpeed = 0;
  private int slowCounts = 0;

  private AlgaeStates curState = AlgaeStates.EMPTY;

  public AlgaeSubsystem(AlgaeIO io) {
    this.io = io;
  }

  public AlgaeStates getState() {
    return curState;
  }

  public void setState(AlgaeStates newState) {
    logger.info("{} -> {}", curState, newState);
    curState = newState;
  }

  public void intakeAlgae() {
    setPct(AlgaeConstants.kIntakingSpeed);
    slowCounts = 0;
    setState(AlgaeStates.EMPTY);
  }

  public void intakeCoral() {
    setPct(AlgaeConstants.kCoralIntakingSpeed);
    slowCounts = 0;
    setState(AlgaeStates.CORAL_INTAKE);
  }

  public void scoreProcessor() {
    setPct(AlgaeConstants.kProcessorScoreSpeed);
  }

  public void scoreBarge() {
    setPct(AlgaeConstants.kBargeScoreSpeed);
  }

  public void scoreCoral() {
    setPct(AlgaeConstants.kCoralScoreSpeed);
  }

  public void holdAlgae() {
    setPct(AlgaeConstants.kHoldSpeed);
  }

  public void holdCoral() {
    setPct(AlgaeConstants.kCoralHoldSpeed);
  }

  public boolean hasAlgae() {
    return curState == AlgaeStates.HAS_ALGAE;
  }

  public boolean hasCoral() {
    return curState == AlgaeStates.HAS_CORAL;
  }

  public boolean hasAlgaeSuperCycle() {
    return curState == AlgaeStates.HAS_ALGAE
        || (FastMath.abs(inputs.velocity) < AlgaeConstants.kSuperCycleHasAlgaeVelThres
            && inputs.isBeamBroken);
  }

  public void setSpeed(double speed) {
    // io.setSpeed(speed);
    // desiredSpeed = speed;
  }

  public void setPct(double pct) {
    io.setPct(pct);
  }

  public double getSpeed() {
    return inputs.velocity;
  }

  public boolean atSpeed() {
    return FastMath.abs(inputs.velocity - desiredSpeed) < AlgaeConstants.kCloseEnough;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput("Algae/state", curState);
    Logger.recordOutput("Algae/setpoint", desiredSpeed);

    switch (curState) {
      case HAS_ALGAE -> {
        if (!inputs.isBeamBroken) {
          setState(AlgaeStates.EMPTY);
          // setPct(0);
          // setSpeed(RotationsPerSecond.of(0));
        }
      }
      case HAS_CORAL -> {
        if (!inputs.isBeamBroken) {
          setState(AlgaeStates.EMPTY);
        }
      }
      case CORAL_INTAKE -> {
        if (inputs.isBeamBroken) {
          if (FastMath.abs(inputs.velocity) < AlgaeConstants.kHasCoralVelThreshold) {
            slowCounts++;
          } else {
            slowCounts = 0;
          }

          if (slowCounts >= AlgaeConstants.kHasCoralCounts) {
            slowCounts = 0;
            holdCoral();
            setState(AlgaeStates.HAS_CORAL);
          }
        }
      }
      case EMPTY -> {
        if (inputs.isBeamBroken) {
          if (FastMath.abs(inputs.velocity) < AlgaeConstants.kHasAlgaeVelThreshold) {
            slowCounts++;
          } else {
            slowCounts = 0;
          }

          if (slowCounts >= AlgaeConstants.kHasAlgaeCounts) {
            slowCounts = 0;
            holdAlgae();
            setState(AlgaeStates.HAS_ALGAE);
          }
        }
      }
      case IDLE -> {}
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  public enum AlgaeStates {
    HAS_ALGAE,
    HAS_CORAL,
    CORAL_INTAKE,
    EMPTY,
    IDLE
  }
}
