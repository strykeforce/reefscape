package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AlgaeSubsystem extends MeasurableSubsystem implements ClosedLoopSpeedSubsystem {
    private final algaeIO io;
    private final AlgaeIOInputs inputs = new AlgaeIOInputs();
    private Angle setpoint = Rotations.of(0.0);
    private AngularVelocity desiredSpeed;

    private enum AlgaeState {
        INIT,
        DETECTING,
        ALGAE_PRESENT,
        NO_ALGAE,
        ERROR
    }

    private AlgaeState currentAlgaeState = AlgaeState.INIT;

    public AlgaeSubsystem(algaeIO io) {
        this.io = io;
        zero();
    }

    public AlgaeState getState() {
        return currentAlgaeState;
    }

    public Angle getPosition() {
        return inputs.position;
    }

    public void setSpeed(AngularVelocity speed) {
        io.setSpeed(speed);
        desiredSpeed = speed;
    }

    public AngularVelocity getSpeed() {
        return inputs.velocity;
    }

    public boolean atSpeed() {
        if (getSpeed().baseUnit().minus(desiredSpeed.baseUnit()) <= AlgaeConstants.kCloseEnough){
            
        }
    }

    public void zero() {
        setpoint = Rotations.of(0);
        io.zero();
        currentAlgaeState = AlgaeState.DETECTING;
    }

    public boolean isFinished() {
        return setpoint.minus(inputs.position).abs(Rotations) <= AlgaeConstants.kCloseEnough.in(Rotations);
    }

    private void updateAlgaeState() {
        if (inputs.algaeDetected) {
            currentAlgaeState = AlgaeState.ALGAE_PRESENT;
        } else if (!inputs.algaeDetected) {
            currentAlgaeState = AlgaeState.NO_ALGAE;
        }
    }

    public void periodic() {
        io.updateInputs(inputs);
        updateAlgaeState();
        Logger.recordOutput("Algae/state", currentAlgaeState.ordinal());
        Logger.recordOutput("Algae/setpoint", setpoint.in(Rotations));

        switch (currentAlgaeState) {
            case ALGAE_PRESENT:
                break;
            case NO_ALGAE:
                break;
        }
    }

    public void registerWith(TelemetryService telemetryService) {
        super.registerWith(telemetryService);
        io.registerWith(telemetryService);
    }

    public Set<Measure> getMeasures() {
        return Set.of(new Measure("State", () -> currentAlgaeState.ordinal()));
    }
}
