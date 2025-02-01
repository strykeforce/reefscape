package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public Angle position = Rotations.of(0.0);
        public AngularVelocity velocity = RotationsPerSecond.of(0.0);
        //something about height zeroing and IO layer?
    
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void registerWith(TelemetryService telemetryService) {}

    public default void setPosition(Angle position) {}

    public default void zero() {}
}











/*
 * To-do: resolve all the comments, implement zero, find out if follower actually means every LOC for one motor does the same
 * thing to the other?, positionrequest never used in IOFX
 */