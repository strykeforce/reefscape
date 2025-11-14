package frc.robot.subsystems.laser;

import edu.wpi.first.wpilibj.RobotController;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LaserSubsystem extends MeasurableSubsystem {

  private final LaserIO io;
  private final LaserIOInputsAutoLogged inputs = new LaserIOInputsAutoLogged();
  private long startTime = 0;

  public LaserSubsystem(LaserIO laserio) {
    this.io = laserio;
  }

  public double getDistance() {
    return io.getDistanceMeters();
  }

  @Override
  public void periodic() {
    startTime = RobotController.getFPGATime();
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("LaserInputs", inputs);
    Logger.recordOutput("Laser/loopTime", (RobotController.getFPGATime() - startTime));
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("distance", "Distance measured by the laser in meters", () -> getDistance()));
  }
}
