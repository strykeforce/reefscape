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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;

/** Implementation of the algaeIO interface using TalonFX motors. */
public class algaeIOFX implements algaeIO {
    private Logger logger;
    private final TalonFX talonFX;
    private StatusSignal<AngularVelocity> currVelocity;
    private StatusSignal<ReverseLimitValue> reverseLimitSwitch;
    private StatusSignal<Angle> currAngle;
    
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
        currAngle = talonFX.getPosition();
    }

    @Override
    public void updateInputs(IOInputs inputs) {
      // inputs.reverseLimitSwitch = reverseLimitSwitch.refresh().getValue().value == 1;
        inputs.velocity = currVelocity.refresh().getValue();
        inputs.location = currAngle.refresh().getValue();
    }

    @Override
    public void setPosition(double position) {
        talonFX.setPosition(position);
    }
    public Angle getPosition() {
        return currAngle.getValue();
    }

    @Override
    public void zero() {
        // not doing one for now
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonFX, true);
    }
}
