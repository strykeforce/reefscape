package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class algaeIOFX implements algaeIO {
  private Logger logger;
  private TalonFX talonFX;
  private AlgaeIOInputs inputs;

  // FX Access objects
  private StatusSignal<AngularVelocity> curVelocity;
  private StatusSignal<ForwardLimitValue> fwdLimitSwitch;
  private StatusSignal<ReverseLimitValue> revLimitSwitch;
  private VelocityVoltage speedRequest = new VelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);

  public algaeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFX(AlgaeConstants.kFxId);
    revLimitSwitch = talonFX.getReverseLimit();
    curVelocity = talonFX.getVelocity();
    fwdLimitSwitch = talonFX.getForwardLimit();
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    BaseStatusSignal.refreshAll(curVelocity, fwdLimitSwitch, revLimitSwitch);
    inputs.velocity = curVelocity.refresh().getValue();
    inputs.isFwdLimitSwitchClosed = fwdLimitSwitch.getValue().value == 1; // FIXME check right value
    inputs.isRevLimitSwitchClosed = revLimitSwitch.getValue().value == 0; // FIXME check right value
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    talonFX.setControl(speedRequest.withVelocity(speed));
  }

  @Override
  public void setPct(double pct) {
    talonFX.setControl(dutyCycleRequest.withOutput(pct));
  }

  public AngularVelocity AngularVelocity() {
    return talonFX.getVelocity().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
