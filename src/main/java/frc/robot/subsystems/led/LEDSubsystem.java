package frc.robot.subsystems.led;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class LEDSubsystem extends MeasurableSubsystem{
    public LEDStates currState = LEDStates.OFF; 
    
    public LEDSubsystem() {
        
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("state", "the current state of the LEDSubsystem", () -> currState.ordinal()));
    }

    public enum LEDStates {
        OFF,
        SOLID,
        FLAMES,
        RAINBOW
    }
}
