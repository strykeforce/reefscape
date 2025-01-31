package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class algaeIOFX implements algaeIO {
  private Logger logger;
  private TalonFXS talonFX;
  private AlgaeIOInputs inputs;

  // FX Access objects
  private TalonFXSConfigurator configurator;
  private StatusSignal<AngularVelocity> currVelocity;
  private StatusSignal<ForwardLimitValue> forwardLimitSwitch;
  private StatusSignal<ReverseLimitValue> reverseLimitSwitch;
  private VelocityVoltage speedRequest = new VelocityVoltage(0).withEnableFOC(false).withSlot(0);

  public algaeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFXS(AlgaeConstants.kFxId);
    reverseLimitSwitch = talonFX.getReverseLimit();
    currVelocity = talonFX.getVelocity();
    forwardLimitSwitch = talonFX.getForwardLimit();
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    BaseStatusSignal.refreshAll(currVelocity);
    inputs.velocity = currVelocity.refresh().getValue();
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    talonFX.setControl(speedRequest.withVelocity(speed));
  }

  public AngularVelocity AngularVelocity() {
    return talonFX.getVelocity().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
