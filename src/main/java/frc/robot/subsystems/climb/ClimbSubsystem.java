package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;

import edu.wpi.first.units.measure.Angle;
import frc.robot.standards.ClosedLoopPosSubsystem;
import frc.robot.subsystems.climb.ClimbArmIO.ClimbArmIOInputs;
import frc.robot.subsystems.example.ExampleIOInputsAutoLogged;
import frc.robot.subsystems.example.ExampleSubsystem.ExampleState;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
    private final ClimbWheelIO wheelIo;
    private final ClimbArmIO armIo;

    private final ClimbWheelIOInputsAutoLogged wheelInputs = new ExampleIOInputsAutoLogged();
    private final ClimbArmIOInputsAutoLogged armInputs = new ExampleIOInputsAutoLogged();

    private Angle setpoint = Rotations.of(0.0);
    private ClimbState curState = ClimbState.INIT;

    public ClimbSubsystem(ClimbWheelIO wheelIo, ClimbArmIO armIo) {
        this.wheelIo = wheelIo;
        this.armIo=armIo;

        zero();
    }

    @Override
    public void zero() {
        armIo.zero();
        curState = ClimbState.ZEROED;
    }

    public enum ClimbState {
        INIT,
        ZEROED
    }
}
