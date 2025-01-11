package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CoralConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class CoralIOFX implements CoralIO {
  // private objects
  private Logger logger;
  private TalonFX talonFx;

  // FX Access objects
  TalonFXConfigurator configurator;
  private MotionMagicVelocityDutyCycle velocityRequest =
      new MotionMagicVelocityDutyCycle(0).withEnableFOC(false).withSlot(0);
  StatusSignal<AngularVelocity> curVelocity;
  StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  StatusSignal<ReverseLimitValue> revLimitSwitch;

  public CoralIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(CoralConstants.kCoralFxId);

    // Config controller
    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(CoralConstants.getFXConfig());

    // Attach status signals
    curVelocity = talonFx.getVelocity();
    fwdLimitSwitch = talonFx.getForwardLimit();
    revLimitSwitch = talonFx.getReverseLimit();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    logger.info("Setting velocity to {} degrees per second", velocity.in(DegreesPerSecond));

    talonFx.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void enableFwdLimitSwitch(boolean enabled) {
    talonFx
        .getConfigurator()
        .apply(CoralConstants.getFXConfig().HardwareLimitSwitch.withForwardLimitEnable(enabled));
  }

  @Override
  public void enableRevLimitSwitch(boolean enabled) {
    talonFx
        .getConfigurator()
        .apply(CoralConstants.getFXConfig().HardwareLimitSwitch.withReverseLimitEnable(enabled));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.velocity = curVelocity.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.refresh().getValue().value == 1;
    inputs.isRevLimitSwitchClosed = revLimitSwitch.refresh().getValue().value == 0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFx, true);
  }
}
