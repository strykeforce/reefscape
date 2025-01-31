package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
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
  private Angle setpoint = Rotations.of(0.0);
  private AngularVelocity desiredSpeed;

  private AlgaeState curState = AlgaeState.EMPTY;

  public AlgaeSubsystem(algaeIO io) {
    this.io = io;
    zero();
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

  private void updateAlgaeState() {
    if (inputs.reverseLimitSwitch.equals(1)) {
      curState = AlgaeState.HAS_ALGAE;
    } else if (!inputs.reverseLimitSwitch.equals(1)) {
      curState = AlgaeState.EMPTY;
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    updateAlgaeState();
    Logger.recordOutput("Algae/state", curState.ordinal());
    Logger.recordOutput("Algae/setpoint", setpoint.in(Rotations));

    switch (curState) {
      case HAS_ALGAE:
        break;
      case EMPTY:
        break;
    }
  }

  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  public enum AlgaeState {
    HAS_ALGAE,
    EMPTY
  }
}
