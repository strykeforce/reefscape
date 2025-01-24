package frc.robot.subsystems.funnel;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import frc.robot.constants.FunnelConstants;
import frc.robot.standards.OpenLoopSubsystem;
import frc.robot.subsystems.example.ExampleIOInputsAutoLogged;

public class FunnelSubsystem extends MeasurableSubsystem implements OpenLoopSubsystem{

    // Private Variables
    private final FunnelIo io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

    public FunnelState curState = FunnelState.HasNotSeenCoral;

    // Constructor
    public FunnelSubsystem(FunnelIo io) {
        this.io = io;
    }

    // Getter/Setter Methods
    public FunnelState getState() {
        return curState;
    }

    @Override
    public void periodic() {
        // Update Inputs
        io.updateInputs(inputs);

        switch (curState){
            case HasSeenCoral:
                break;
            case HasNotSeenCoral:
                break;
        }

        // Log Outputs
        Logger.recordOutput("Funnel/curState", curState);
    }

    // Grapher
    @Override
    public void registerWith(TelemetryService telemetryService) {
        io.registerWith(telemetryService);
        super.registerWith(telemetryService);
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("State", () -> curState.ordinal()));
    }

    // State Enum
    public enum FunnelState {
        HasSeenCoral,
        HasNotSeenCoral
    }

    @Override
    public void setPercent(double pct) {
        io.setPct(pct);
    }

    public void StartMotor() {
        setPercent(FunnelConstants.kFunnelPercentOutput);
    }

    public void StopMotor() {
        setPercent(0.0);
    }
    
}
