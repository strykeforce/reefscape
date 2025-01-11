package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ClimbConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final ClimbArmIO io;
  private final ClimbWheelIO wheelIo;

  private final ClimbArmIOInputsAutoLogged inputs = new ClimbArmIOInputsAutoLogged();

  private Angle setpoint = Rotations.of(0.0);
  private ClimbState curState = ClimbState.INIT;

  public ClimbSubsystem(ClimbArmIO io, ClimbWheelIO wheelIo) {
    this.io = io;
    this.wheelIo = wheelIo;

    zero();
  }

  @Override
  public Angle getPosition() {
    return inputs.position;
  }

  @Override
  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  @Override
  public boolean isFinished() {
    return setpoint.minus(inputs.position).abs(Rotations)
        <= ClimbConstants.kArmCloseEnough.in(Rotations);
  }

  @Override
  public void zero() {
    io.zero();

    curState = ClimbState.ZEROED;
  }

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
    Logger.recordOutput("Coral/setpoint", setpoint.in(Rotations));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    wheelIo.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public enum ClimbState {
    INIT,
    ZEROED
  }
}
