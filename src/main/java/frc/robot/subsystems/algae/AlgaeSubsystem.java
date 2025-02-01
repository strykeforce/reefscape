package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.algae.algaeIO.AlgaeIOInputs;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AlgaeSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  private org.slf4j.Logger logger = LoggerFactory.getLogger(AlgaeSubsystem.class);

  private final algaeIO io;
  private final AlgaeIOInputs inputs = new AlgaeIOInputs();
  private AngularVelocity desiredSpeed;

  private AlgaeStates curState = AlgaeStates.IDLE;

  public AlgaeSubsystem(algaeIO io) {
    this.io = io;
  }

  public AlgaeStates getState() {
    return curState;
  }

  public void setState(AlgaeStates newState) {
    logger.info("{} -> {}", curState, newState);
    curState = newState;
  }

  public void intake() {
    setSpeed(AlgaeConstants.kIntakingSpeed);
  }

  public void scoreProcessor() {
    setSpeed(AlgaeConstants.kProcessorScoreSpeed);
  }

  public void scoreBarge() {
    setSpeed(AlgaeConstants.kBargeScoreSpeed);
  }

  public void hold() {
    setSpeed(AlgaeConstants.kHoldSpeed);
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    io.setSpeed(speed);
    desiredSpeed = speed;
  }

  public void setPct(double pct) {
    io.setPct(pct);
  }

  @Override
  public AngularVelocity getSpeed() {
    return inputs.velocity;
  }

  @Override
  public boolean atSpeed() {
    return inputs.velocity.minus(desiredSpeed).abs(RotationsPerSecond)
        < AlgaeConstants.kCloseEnough.in(RotationsPerSecond);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Algae/state", curState);
    Logger.recordOutput("Algae/setpoint", desiredSpeed.in(RotationsPerSecond));

    switch (curState) {
      case HAS_ALGAE -> {
        if (!inputs.isFwdLimitSwitchClosed) {
          setState(AlgaeStates.EMPTY);
          setSpeed(RotationsPerSecond.of(0));
        }
      }
      case EMPTY -> {
        if (inputs.isRevLimitSwitchClosed) { // FIXME: correct?
          hold();
          setState(AlgaeStates.HAS_ALGAE);
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
    EMPTY,
    IDLE
  }
}
