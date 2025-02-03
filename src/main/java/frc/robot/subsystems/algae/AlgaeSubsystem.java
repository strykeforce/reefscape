//PING EPICPIGGUY ON DISCORD IF QUESTIONS


package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.algae.algaeIO.AlgaeIOInputs;

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
        ALGAE_PRESENT,
        NO_ALGAE
    }

    private AlgaeState currentAlgaeState = AlgaeState.NO_ALGAE;

    public AlgaeSubsystem(algaeIO io) {
        this.io = io;
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
        return inputs.velocity.equals(desiredSpeed);
    }
    //not sure if i need this
    // public void zero() {
    //     setpoint = Rotations.of(0);
    //     io.zero();
    // }

    private void updateAlgaeState() {
        if (inputs.reverseLimitSwitch.equals(1)) {
            currentAlgaeState = AlgaeState.ALGAE_PRESENT;
        } else if (!inputs.reverseLimitSwitch.equals(1)) {
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
