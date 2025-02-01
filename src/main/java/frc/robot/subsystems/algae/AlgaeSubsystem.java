package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.algae.algaeIO.AlgaeIOInputs;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AlgaeSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  private final algaeIO io;
  private final AlgaeIOInputs inputs = new AlgaeIOInputs();
  private AngularVelocity desiredSpeed;

  private AlgaeState curState = AlgaeState.IDLE;

  public AlgaeSubsystem(algaeIO io) {
    this.io = io;
  }

  public AlgaeState getState() {
    return curState;
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    io.setSpeed(speed);
    desiredSpeed = speed;
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
          curState = AlgaeState.EMPTY;
        }
      }
      case EMPTY -> {
        if (inputs.isRevLimitSwitchClosed) {
          curState = AlgaeState.HAS_ALGAE;
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

  public enum AlgaeState {
    HAS_ALGAE,
    EMPTY,
    IDLE
  }
}
