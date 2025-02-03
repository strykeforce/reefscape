//PING EPICPIGGUY ON DISCORD IF QUESTIONS

package frc.robot.subsystems.algae;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.ExampleConstants;

public class algaeIOFX implements algaeIO {
  private Logger logger;
  private final TalonFX talonFX;
  private AlgaeIOInputs inputs;

  // FX Access objects
    TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<AngularVelocity> currVelocity;
  StatusSignal<ForwardLimitValue> forwardLimitSwitch;
  StatusSignal<ReverseLimitValue> reverseLimitSwitch;
  VelocityVoltage speedRequest = new VelocityVoltage(0).withEnableFOC(false).withSlot(0);

  public algaeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFX(AlgaeConstants.kFxId);
    reverseLimitSwitch = talonFX.getReverseLimit();
    currVelocity = talonFX.getVelocity();
    forwardLimitSwitch = talonFX.getForwardLimit();
  }

  public void updateInputs(AlgaeIOInputs inputs) {
    BaseStatusSignal.refreshAll();
    inputs.velocity = currVelocity.refresh().getValue();
  }

  public void setSpeed(AngularVelocity speed) {
    talonFX.setControl(speedRequest.withVelocity(speed));
  }

  public AngularVelocity AngularVelocity() {
    return talonFX.getVelocity().getValue();
  }

  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
