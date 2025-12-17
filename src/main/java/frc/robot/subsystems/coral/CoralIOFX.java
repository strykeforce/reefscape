package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CoralConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;

public class CoralIOFX implements CoralIO, Checkable {
  // private objects
  private Logger logger;

  @HealthCheck
  @Timed(
      percentOutput = {0.1, 1},
      duration = 2)
  private TalonFXS talonFx;

  // FX Access objects
  private TalonFXSConfigurator configurator;
  private MotionMagicVelocityDutyCycle velocityRequest =
      new MotionMagicVelocityDutyCycle(0).withEnableFOC(false).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  private StatusSignal<AngularVelocity> curVelocity;
  private StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  private StatusSignal<ReverseLimitValue> revLimitSwitch;

  public CoralIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFXS(CoralConstants.kCoralFxId);

    // Config controller
    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXSConfiguration()); // Factory default motor controller
    configurator.apply(CoralConstants.getFXConfig());

    // Attach status signals
    curVelocity = talonFx.getVelocity();
    fwdLimitSwitch = talonFx.getForwardLimit();
    revLimitSwitch = talonFx.getReverseLimit();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // logger.info("Setting velocity to {} rots per second", velocity.in(RotationsPerSecond));

    talonFx.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPct(double percentOutput) {
    talonFx.setControl(dutyCycleRequest.withOutput(percentOutput));
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
    BaseStatusSignal.refreshAll(curVelocity, fwdLimitSwitch, revLimitSwitch);
    inputs.velocity = curVelocity.getValue();
    inputs.isFwdBeamBroken = fwdLimitSwitch.getValue().value == 0;
    inputs.isRevBeamBroken = revLimitSwitch.getValue().value == 0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFx, true);
  }

  @Override
  public String getName() {
    return "Coral";
  }
}
