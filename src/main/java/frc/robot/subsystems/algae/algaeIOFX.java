package frc.robot.subsystems.algae;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.IOInputsAutoLogged;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.AlgaeConstants;

public class algaeIOFX implements algaeIO {
  private Logger logger;
  private final TalonFX talonFX;
  private IOInputs inputs;

  // FX Access objects
  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<AngularVelocity> currVelocity;
  StatusSignal<ReverseLimitValue> reverseLimitSwitch;

  public algaeIOFX() {
    final IOInputsAutoLogged inputs = new IOInputsAutoLogged();
    logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFX(AlgaeConstants.kFxId);
    reverseLimitSwitch = talonFX.getReverseLimit();
    currVelocity = talonFX.getVelocity();
    reverseLimitSwitch = talonFX.getReverseLimit();
  }

  public void updateInputs(IOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
  }
  public void setPosition(double position) {
        // Set the Talon FX to position control mode and set the target position
        talonFX.setPosition(position);
  }

  public void zero() {
  }

  public void registerWith(TelemetryService telemetryService) {}
}
