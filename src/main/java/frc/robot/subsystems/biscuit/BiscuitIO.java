package frc.robot.subsystems.biscuit;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface BiscuitIO {

  @AutoLog
  public class BiscuitIOInputs {
    public Angle position = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public boolean fwdLimitSwitchOpen = false;
  }

  public default void setPosition(Angle position) {}

  public default void updateInputs(BiscuitIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetry) {}

  public default void zero() {}
}
