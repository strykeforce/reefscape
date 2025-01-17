package frc.robot.subsystems.funnel;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.FunnelConstants;

public class FunnelIOFX implements FunnelIo{
    // Private Objects
    private Logger logger;
    private TalonFX talonfx;

    // FX Acces Objects
    TalonFXConfigurator configurator;
    StatusSignal<AngularVelocity> curVelocity;
    StatusSignal<ReverseLimitValue> curRevLimit;

    private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(false);

    public FunnelIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
        talonfx = new TalonFX(FunnelConstants.FunnelFxId);

        // Config controller
        configurator = talonfx.getConfigurator();
        configurator.apply(new TalonFXConfiguration());
        configurator.apply(FunnelConstants.getFXConfig());

        // Attach status signals
        curVelocity = talonfx.getVelocity();
        curRevLimit = talonfx.getReverseLimit();
    }

    @Override
    public void setPct(double percentOutput){
        talonfx.setControl(dutyCycleRequest.withOutput(percentOutput));
    }

    @Override
    public void updateInputs(FunnelIoInputs inputs) {
        inputs.velocity = curVelocity.refresh().getValue();
        inputs.revBeamOpen = curRevLimit.refresh().getValue().value == 1;
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(talonfx, true);
    }
}
