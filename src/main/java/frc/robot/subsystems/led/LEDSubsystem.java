package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends MeasurableSubsystem{
    private LEDIO io;
    private LEDPattern base = LEDPattern.solid(Color.kBlack);
    private LEDPattern coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
    private LEDPattern algea = LEDPattern.steps(Map.of(0, LEDConstants.kHasAlgea, LEDConstants.stripLength/3, Color.kBlack));

    private LEDPattern currentLimiting = LEDPattern.solid(LEDConstants.kCurrentLimiting).blink(Seconds.of(1), Seconds.of(2));
    private LEDPattern autoplace = LEDPattern.steps(Map.of(LEDConstants.stripLength/3*2, LEDConstants.kAutoPlacing)).blink(Seconds.of(0.25));
    public LEDStates currState = LEDStates.OFF;
    public CoralStates coralState = CoralStates.NO_PIECE; 
    public boolean autoPlacing = false;
    public boolean hasAlgae = false;
    public boolean isLimiting = false;
    
    public LEDSubsystem(LEDIO io) {
        this.io = io;
    }

    private void buildBase() {
        switch (coralState) {
            case IN_FUNNEL:
                coral = LEDPattern.solid(LEDConstants.kCoralInFunnel);
                break;
            case IN_ROBOT:
                coral = LEDPattern.solid(LEDConstants.kCoralInRobot);
                break;
            case NO_PIECE:
                coral = LEDPattern.solid(LEDConstants.kCoralNotInRobot);
                break;
        }
        algea = LEDPattern.steps(Map.of(0, LEDConstants.kHasAlgea, LEDConstants.stripLength/3, Color.kBlack));
        
    }

    public void periodic() {
        switch (currState) {
            case OFF:
                break;
            case NORMAL:
                if (autoPlacing) {
                    base = autoplace.overlayOn(base);
                }
                if (isLimiting) {
                    base = currentLimiting.overlayOn(base);
                }
                io.setStrip(base);
                break;
            case CLIMB:
                break;
        }
        io.updateLEDs();
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
        IN_FUNNEL,
        IN_ROBOT,
        NO_PIECE,
    }
}
