package frc.robot.standards;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.Timer;

public class StateMachineSubsystem<S extends Enum<S>> extends MeasurableSubsystem {
    protected S curState;
    protected S intiialState;
    protected boolean isInitialized;
    protected double lastStateTransition = Timer.getFPGATimestamp();
    protected double timeInState = 0.0;
    protected double curTime = 0.0;
    
    public StateMachineSubsystem(S initialState) {
        isInitialized = false;
        this.intiialState = initialState;
    }

    public S getState() {
        return curState;
    }

    public void setState(S newState) {
        transitionStates(newState);
    }

    @Override
    public void periodic() {
        if(!isInitialized) {
            transitionStates(intiialState);
            isInitialized = true;  
        } 

        S nextState = shouldTransitionStates();
        transitionStates(nextState);
        executeState();
    }

    /* Called every robot loop to check if should transition states
     * returns the current state if should not transition */
    protected S shouldTransitionStates() {
        return null;
    }

    /* Called every robot loop to transition from curState to newState
     * Handles exit logic of curState and entry logic of newState */
    protected void transitionStates(S newState) {
        if(newState == curState) return;
        curTime = Timer.getFPGATimestamp();
        timeInState = curTime - lastStateTransition;
        lastStateTransition = curTime;

    }

    /* Called every robot loop to execute actions that happen while in the current state this is run after checking for state transitions */
    protected void executeState() {

    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of();
    }

    


    
}
