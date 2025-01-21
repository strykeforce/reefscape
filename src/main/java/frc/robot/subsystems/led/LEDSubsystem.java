package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends MeasurableSubsystem{
    private LEDIO io;
    private LEDPattern base = LEDPattern.steps(map.of(0, Color.kAqua, ));
    LEDPattern red = LEDPattern.solid(Color.kRed);
    public LEDStates currState = LEDStates.OFF;
    public CoralStates coralState = CoralStates.NO_PIECE; 
    public boolean autoPlacing = false;
    public boolean hasAlgae = false;
    public boolean isLimiting = false;
    
    public LEDSubsystem(LEDIO io) {
        this.io = io;
        red.blink(Seconds.of(1), Seconds.of(2));
        
    }

    public void periodic() {
        red.blink(Seconds.of(1), Seconds.of(2));
        if (isLimiting) {
            red.overlayOn(base);
        }
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("State", "the current Overall state of the LEDSubsystem", () -> currState.ordinal()),
            new Measure("CoralState", "the current Coral state of the LEDSubsystem", () -> coralState.ordinal()));
    }

    public enum LEDStates {
        OFF,
        NORMAL,
        CLIMB
    }

    public enum CoralStates {
        HAS_PIECE,
        IN_MECH,
        NO_PIECE,
    }
}
