package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CoralConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class CoralSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  private AngularVelocity setpoint = RotationsPerSecond.of(0.0);
  private CoralState curState = CoralState.IDLE;
  private org.slf4j.Logger logger = LoggerFactory.getLogger(CoralSubsystem.class);

  public CoralSubsystem(CoralIO io) {
    this.io = io;
  }

  public CoralState getState() {
    return curState;
  }

  @Override
  public AngularVelocity getSpeed() {
    return inputs.velocity;
  }

  public void setState(CoralState state) {
    logger.info("{} -> {}", curState, state);
    this.curState = state;
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    setpoint = speed;
    io.setVelocity(speed);
  }

  @Override
  public boolean atSpeed() {
    return setpoint.minus(inputs.velocity).abs(RotationsPerSecond)
        <= CoralConstants.kCloseEnough.in(RotationsPerSecond);
  }

  public boolean isEnterBeamBroken() {
    return inputs.isFwdBeamBroken;
  }

  public boolean isExitBeamBroken() {
    return inputs.isRevBeamBroken;
  }

  public void intake() {
    io.enableRevLimitSwitch(true);
    setSpeed(CoralConstants.kIntakingSpeed);
    setState(CoralState.INTAKING);
  }

  public void eject() {
    io.enableRevLimitSwitch(false);
    setSpeed(CoralConstants.kEjectingSpeed);
    setState(CoralState.EJECTING);
  }

  // Periodic Function
  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    // State Machine
    switch (curState) {
      case IDLE -> {}
      case INTAKING -> {
        if (isEnterBeamBroken()) {
          setState(CoralState.CORAL_LOADING);
        }
      }
      case CORAL_LOADING -> {
        if (isExitBeamBroken()) {
          setState(CoralState.HAS_CORAL);
        }
      }
      case HAS_CORAL -> {}
      case EJECTING -> {
        if (!isExitBeamBroken()) {
          setState(CoralState.EMPTY);
        }
      }
      case EMPTY -> {}
      default -> {}
    }

    // Log Outputs
    Logger.recordOutput("Coral/curState", curState);
    Logger.recordOutput("Coral/setpoint", setpoint.in(RotationsPerSecond));
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

  public enum CoralState {
    IDLE,
    HAS_CORAL,
    CORAL_LOADING,
    INTAKING,
    EJECTING,
    EMPTY
  }
}
