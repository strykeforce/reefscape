package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.FunnelConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class FunnelIOFXS implements FunnelIO {
  // Private Objects
  private Logger logger;
  private TalonFXS talonfxs;

  // FX Acces Objects
  TalonFXSConfigurator configurator;
  StatusSignal<AngularVelocity> curVelocity;
  StatusSignal<ReverseLimitValue> curRevLimit;

  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(false);

  public FunnelIOFXS() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonfxs = new TalonFXS(FunnelConstants.FunnelFxsId);

    // Config controller
    configurator = talonfxs.getConfigurator();
    configurator.apply(new TalonFXSConfiguration());
    configurator.apply(FunnelConstants.getFXSConfig());

    // Attach status signals
    curVelocity = talonfxs.getVelocity();
    curRevLimit = talonfxs.getReverseLimit();
  }

  @Override
  public void setPct(double percentOutput) {
    talonfxs.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    BaseStatusSignal.refreshAll(curVelocity, curRevLimit);
    inputs.velocity = curVelocity.getValue();
    inputs.isRevBeamBroken = curRevLimit.getValue().value == 0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonfxs, true);
  }
}
