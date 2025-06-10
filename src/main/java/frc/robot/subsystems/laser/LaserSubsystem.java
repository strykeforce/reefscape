package frc.robot.subsystems.laser;

import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LaserSubsystem extends MeasurableSubsystem {

  private final LaserIO laserio;

  public LaserSubsystem(LaserIO laserio) {
    this.laserio = laserio;
  }

  public double getDistance() {
    return laserio.getDistanceMeters();
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("distance", "Distance measured by the laser in meters", () -> getDistance()));
  }
}
