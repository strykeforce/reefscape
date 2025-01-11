package frc.robot.subsystems.funnel;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import frc.robot.standards.OpenLoopSubsystem;

public class FunnelSubsystem extends MeasurableSubsystem implements OpenLoopSubsystem{

    // Private Variables
    private final FunnelIo io;

    public FunnelState curState = FunnelState.Idle;

    // Constructor
    public FunnelSubsystem(FunnelIo io){
        this.io = io;
    }

    public FunnelState getState(){
        return curState;
    }

    @Override
    public void periodic(){
        switch (curState){
            case Idle:
                break;
            case PickingUp:
                break;
            case Held:
                break;
        }
    }

    @Override
    public Set<Measure> getMeasures() {
            return null;
    }

    public enum FunnelState{
        Idle,
        PickingUp,
        Held
    }

    @Override
    public void setPercent(double pct) {

    }
    
}
