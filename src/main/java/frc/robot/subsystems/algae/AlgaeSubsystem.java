package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.standards.ClosedLoopSpeedSubsystem;
import frc.robot.subsystems.algae.algaeIO.AlgaeIOInputs;

public class AlgaeSubsystem implements ClosedLoopSpeedSubsystem {
    private final algaeIO io;
    private final IOInputsAutoLogged inputs = new IOInputsAutoLogged();
    private Angle setpoint = Rotations.of(0.0);  // Initial setpoint for position
    private AngularVelocity desiredSpeed = AngularVelocity.ZERO;  // Initial desired speed

    public AlgaeSubsystem(algaeIO io) {
        this.io = io;
    }

    // Get the current position from the io object
    
    public Angle getPosition() {
        return AlgaeIOInputs.position;
    }

    // Set the speed (velocity) of the motor
    
    public void setSpeed(AngularVelocity speed) {
        io.set(speed);
        this.desiredSpeed = speed;  // Keep track of the desired speed
    }

    // Get the current speed (velocity) of the motor
    
    public AngularVelocity getSpeed() {
        return io.getSpeed();  // Assuming algaeIO provides the speed
    }

    // Check if the subsystem is at the desired speed
    
    public boolean atSpeed() {
        // Check if the current speed is within a tolerance of the desired speed
        return getSpeed().minus(desiredSpeed).abs() <= AlgaeConstants.kSpeedTolerance.in(AngularVelocity.class);
    }

    // Zero the subsystem (reset position to zero)
    
    public void zero() {
        setpoint = Rotations.of(0);  // Reset the setpoint to zero
        io.zero();  // Call the zero method in algaeIO (to reset hardware)
    }

    // Check if the subsystem has reached the setpoint position
    public boolean isFinished() {
        return setpoint.minus(getPosition()).abs(Rotations) <= AlgaeConstants.kCloseEnough.in(Rotations);
    }

    // Periodic function that runs repeatedly during the robot's operation
    public void periodic() {
        io.updateInputs(inputs);  // Update the inputs from the hardware
        Logger.processInputs(getName(), inputs);  // Log inputs
        Logger.recordOutput("Algae/setpoint", setpoint.in(Rotations));  // Log setpoint position
    }

    // Register the subsystem with a telemetry service for monitoring
    public void registerWith(TelemetryService telemetryService) {
        io.registerWith(telemetryService);  // Register io with the telemetry service
    }
}