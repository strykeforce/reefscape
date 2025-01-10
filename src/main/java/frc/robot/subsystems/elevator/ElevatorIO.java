package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ElevatorIO {
    public double position = 0.0;
    public double velocityLeft = 0.0;
    public double velocityRight = 0.0;

    public default void updateInputs(ExiterIOInputs inputs) {}

    public default void registerWith(TelemetryService telemetryService) {}
    
}
