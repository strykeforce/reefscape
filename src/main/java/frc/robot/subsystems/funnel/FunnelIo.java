package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface FunnelIo {

    @AutoLog
    public static class FunnelIoInputs{

    }

    public default void updateInputs(FunnelIoInputs inputs) {}

    public default void registerWith(TelemetryService telemetryService) {}
}
