package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CoralConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoringLevel;
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
  private Timer loopTimer = new Timer();
  private long startTime = 0;

  public CoralSubsystem(CoralIO io) {
    this.io = io;
    loopTimer.start();
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

  public void setAutoPreload() {
    intake();
    setState(CoralState.HAS_CORAL);
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    setpoint = speed;
    io.setVelocity(speed);
  }

  public void setPct(double percentOutput) {
    io.setPct(percentOutput);
  }

  public void enableEjectBeam(boolean enable) {
    io.enableFwdLimitSwitch(enable);
  }

  @Override
  public boolean atSpeed() {
    return setpoint.minus(inputs.velocity).abs(RotationsPerSecond)
        <= CoralConstants.kCloseEnough.in(RotationsPerSecond);
  }

  public boolean isEnterBeamBroken() {
    return inputs.isRevBeamBroken;
  }

  public boolean isExitBeamBroken() {
    return inputs.isFwdBeamBroken;
  }

  public boolean hasCoral() {
    switch (curState) {
      case HAS_CORAL, EJECTING -> {
        return true;
      }
      default -> {
        return false;
      }
    }
  }

  public boolean hasCoralAuton() {
    switch (curState) {
      case HAS_CORAL, EJECTING, CORAL_LOADING -> {
        return true;
      }
      default -> {
        return false;
      }
    }
  }

  public void intake() {
    io.enableFwdLimitSwitch(true);
    // setSpeed(CoralConstants.kIntakingSpeed);
    setPct(.5);
    setState(CoralState.INTAKING);
  }

  public void eject(ScoringLevel level) {
    io.enableFwdLimitSwitch(false);
    // setSpeed(CoralConstants.kEjectingSpeed);

    switch (level) {
      case L1, L2 -> setPct(0.8);
      case L3 -> setPct(0.75);
      case L4 -> setPct(1);
    }
    setState(CoralState.EJECTING);
  }

  public void stop() {
    setPct(0);
  }

  // Periodic Function
  @Override
  public void periodic() {
    loopTimer.reset();
    loopTimer.start();
    startTime = RobotController.getFPGATime();
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
    Logger.recordOutput("Coral/loopTime", (RobotController.getFPGATime() - startTime));
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

  public void healthCheck() {
    setState(CoralState.IDLE);
    setPct(0);
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
