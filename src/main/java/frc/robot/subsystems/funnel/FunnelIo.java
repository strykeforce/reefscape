package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FunnelIo {

    @AutoLog
    public static class FunnelIoInputs{
        public AngularVelocity velocity;
        // Open = beam isnt broken
        public boolean forBeamOpen = false;
        public boolean beamEnabled = true;
    }

    public default void setPct(double percentOutput) {}

    public default void updateInputs(FunnelIoInputs inputs) {}

    public default void registerWith(TelemetryService telemetryService) {}
}
