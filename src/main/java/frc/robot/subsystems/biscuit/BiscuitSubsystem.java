package frc.robot.subsystems.biscuit;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.*;
import frc.robot.constants.BiscuitConstants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class BiscuitSubsystem extends MeasurableSubsystem {

  private BiscuitIO io;
  private BiscuitIOInputsAutoLogged inputs = new BiscuitIOInputsAutoLogged();
  private Angle setPoint;

  public void setPosition(Angle position) {
    io.setPosition(position);
    setPoint = position;
  }

  public Angle getPosition(Angle position) {
    return inputs.position;
  }

  public AngularVelocity getVelocity(AngularVelocity velocity) {
    return inputs.velocity;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  @Override
  public void registerWith(TelemetryService telemetry) {
    super.registerWith(telemetry);
    io.registerWith(telemetry);
  }

  public boolean isFinished() {
    return setPoint.minus(inputs.position).abs(Rotations) <= BiscuitConstants.kCloseEnough;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Is Biscuit Finished", () -> isFinished() ? 1.0 : 0.0));
  }

  public void zero() {
    io.zero();
  }
}
