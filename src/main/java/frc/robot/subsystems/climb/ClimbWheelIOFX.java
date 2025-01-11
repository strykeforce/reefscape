package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ClimbConstants;

public class ClimbWheelIOFX implements ClimbWheelIO {
  private Logger logger;
  private TalonFX talonFx;

  TalonFXConfigurator configurator;
  private MotionMagicVelocityDutyCycle velocityRequest = new MotionMagicVelocityDutyCycle(0).withEnableFOC(false).withSlot(0);
  StatusSignal<AngularVelocity> curVelocity;

  public ClimbWheelIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(ClimbConstants.kWheelFxId);

    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(ClimbConstants.getWheelFXConfig());

    curVelocity = talonFx.getVelocity();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    logger.info("Setting velocity to {} degrees per second", velocity.in(DegreesPerSecond));

      talonFx.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void updateInputs(ClimbWheelIOInputs inputs) {
      inputs.velocity = curVelocity.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
      telemetryService.register(talonFx, true);
  }
}
