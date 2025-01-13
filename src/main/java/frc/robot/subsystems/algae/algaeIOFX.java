package frc.robot.subsystems.algae;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;

/** Implementation of the algaeIO interface using TalonFX motors. */
public class algaeIOFX implements algaeIO {
    private Logger logger;
    private final TalonFX talonFX;
    private StatusSignal<AngularVelocity> currVelocity;
    private StatusSignal<ReverseLimitValue> reverseLimitSwitch;
    
    // FX Access objects
    TalonFXConfigurator configurator;
    private MotionMagicDutyCycle positionRequest = 
        new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);

    // Constructor to initialize the TalonFX motor controller
    public algaeIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
        talonFX = new TalonFX(AlgaeConstants.kFxId);
        reverseLimitSwitch = talonFX.getReverseLimit();
        currVelocity = talonFX.getVelocity();
    }

    @Override
    public void updateInputs(IOInputs inputs) {
        // Update the velocity input in the IOInputs structure
        inputs.velocity = currVelocity.refresh().getValue();
    }

    @Override
    public void setPosition(double position) {
        // In case you want to use the position control in your system
        talonFX.setPosition(position);
    }

    @Override
    public void zero() {
        // This can reset the encoder or clear other state as needed
        talonFX.clearStickyFaults();
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonFX, true);
    }
}
