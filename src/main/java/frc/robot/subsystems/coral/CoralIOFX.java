package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CoralConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class CoralIOFX implements CoralIO {
  private Logger logger;
  private TalonFXS talonFxs;

  // FX Access objects
  TalonFXSConfigurator configurator;
  private MotionMagicVelocityDutyCycle velocityRequest =
      new MotionMagicVelocityDutyCycle(0).withEnableFOC(false).withSlot(0);
  StatusSignal<AngularVelocity> curVelocity;
  StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  StatusSignal<ReverseLimitValue> revLimitSwitch;

  public CoralIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFxs = new TalonFXS(CoralConstants.kCoralFxId);

    // Config controller
    configurator = talonFxs.getConfigurator();
    configurator.apply(new TalonFXSConfiguration()); // Factory default motor controller
    configurator.apply(CoralConstants.getFXConfig());

    // Attach status signals
    curVelocity = talonFxs.getVelocity();
    fwdLimitSwitch = talonFxs.getForwardLimit();
    revLimitSwitch = talonFxs.getReverseLimit();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    logger.info("Setting velocity to {} degrees per second", velocity.in(DegreesPerSecond));

    talonFxs.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void enableFwdLimitSwitch(boolean enabled) {
    talonFxs
        .getConfigurator()
        .apply(CoralConstants.getFXConfig().HardwareLimitSwitch.withForwardLimitEnable(enabled));
  }

  @Override
  public void enableRevLimitSwitch(boolean enabled) {
    talonFxs
        .getConfigurator()
        .apply(CoralConstants.getFXConfig().HardwareLimitSwitch.withReverseLimitEnable(enabled));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    BaseStatusSignal.refreshAll(curVelocity, fwdLimitSwitch, revLimitSwitch);
    inputs.velocity = curVelocity.getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFxs, true);
  }
}
