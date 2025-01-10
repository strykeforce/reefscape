package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CoralConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class CoralSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  private AngularVelocity setpoint = RotationsPerSecond.of(0.0);
  private CoralState curState = CoralState.INIT;

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

  public boolean isBeamBroken() {
    return false; // FIXME when we get the robot
  }

  // Periodic Function
  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(inputs);

    // State Machine
    switch (curState) {
      case INIT:
        break;
      case ZEROED:
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Coral/curState", curState.ordinal());
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
    INIT,
    ZEROED
  }
}
