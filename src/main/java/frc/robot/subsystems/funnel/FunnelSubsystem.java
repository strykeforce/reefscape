package frc.robot.subsystems.funnel;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import frc.robot.constants.FunnelConstants;
import frc.robot.standards.OpenLoopSubsystem;

public class FunnelSubsystem extends MeasurableSubsystem implements OpenLoopSubsystem{

    // Private Variables
    private final FunnelIo io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
    private float totalBreaks = 0;

    public FunnelState curState = FunnelState.HasNotSeenCoral;

    // Constructor
    public FunnelSubsystem(FunnelIo io) {
        this.io = io;
    }

    // Getter/Setter Methods
    public FunnelState getState() {
        return curState;
    }

    public boolean hasCoral() {
      // return true;
      return curState == FunnelState.HasSeenCoral;
    }

    @Override
    public void periodic() {
        // Update Inputs
        io.updateInputs(inputs);
        Logger.processInputs("funnelInputs", inputs);

        switch (curState){
            case HasSeenCoral:
                break;
            case HasNotSeenCoral:
                if(inputs.fwdBeamOpen == false){
                    totalBreaks += 1;
                }else{
                    totalBreaks = 0;
                }

                if(totalBreaks >= 3){
                    stopMotor();
                    curState = FunnelState.HasSeenCoral;
                }
                break;
        }

        // Log Outputs
        Logger.recordOutput("Funnel/curState", curState);
        Logger.recordOutput("Funnel/totalBreaks", totalBreaks);
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

    public void startMotor() {
        setPercent(FunnelConstants.kFunnelPercentOutput);
        io.enableFwdLimitSwitch(false);
    }

    public void ClearCoral() {
        curState = FunnelState.HasNotSeenCoral;
        setPercent(FunnelConstants.kFunnelPercentOutput);
        io.enableFwdLimitSwitch(true);
    }
 
    public void stopMotor() {
        setPercent(0.0);
        io.enableFwdLimitSwitch(false);
    }
}
