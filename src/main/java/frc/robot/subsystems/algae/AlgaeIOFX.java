package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.constants.AlgaeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.Checkable;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;

public class AlgaeIOFX implements AlgaeIO, Checkable {
  private Logger logger;

  @HealthCheck
  @Timed(
      percentOutput = {0.5, 0.04, -1},
      duration = 2)
  private TalonFXS talonFXS;

  private AlgaeIOInputs inputs;

  private TalonFXSConfigurator configurator;

  // FX Access objects
  private StatusSignal<AngularVelocity> curVelocity;
  private StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  private StatusSignal<ReverseLimitValue> revLimitSwitch;
  private VelocityVoltage speedRequest = new VelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);
  private StatusSignal<Current> statorCurrent;

  public AlgaeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFXS = new TalonFXS(AlgaeConstants.kFxId);

    configurator = talonFXS.getConfigurator();
    configurator.apply(new TalonFXSConfiguration()); // Factory default motor controller
    configurator.apply(AlgaeConstants.getFXConfig());

    fwdLimitSwitch = talonFXS.getForwardLimit();
    revLimitSwitch = talonFXS.getReverseLimit();
    curVelocity = talonFXS.getVelocity();
    statorCurrent = talonFXS.getStatorCurrent();
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    BaseStatusSignal.refreshAll(curVelocity, fwdLimitSwitch, revLimitSwitch, statorCurrent);
    inputs.velocity = curVelocity.getValueAsDouble();
    inputs.isBeamBroken = fwdLimitSwitch.getValue().value == 0;
    inputs.isCoralBeamBroken = revLimitSwitch.getValue().value == 1;
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
  }

  @Override
  public void setSpeed(double speed) {
    talonFXS.setControl(speedRequest.withVelocity(speed));
  }

  @Override
  public void setPct(double pct) {
    talonFXS.setControl(dutyCycleRequest.withOutput(pct));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFXS, true);
  }

  @Override
  public String getName() {
    return "Algae";
  }
}
