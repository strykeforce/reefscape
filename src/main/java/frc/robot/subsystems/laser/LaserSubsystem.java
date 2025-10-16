package frc.robot.subsystems.laser;

import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LaserSubsystem extends MeasurableSubsystem {

  private final LaserIO io;
  private final LaserIOInputsAutoLogged inputs = new LaserIOInputsAutoLogged();

  public LaserSubsystem(LaserIO laserio) {
    this.io = laserio;
  }

  public double getDistance() {
    return io.getDistanceMeters();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("LaserInputs", inputs);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("distance", "Distance measured by the laser in meters", () -> getDistance()));
  }
}
