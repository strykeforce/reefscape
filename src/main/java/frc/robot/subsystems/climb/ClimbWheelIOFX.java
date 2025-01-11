package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ClimbConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ClimbWheelIOFX implements ClimbWheelIO {
  private Logger logger;
  private TalonFX talonFx;

  TalonFXConfigurator configurator;

  public ClimbWheelIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(ClimbConstants.kWheelFxId);

    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(ClimbConstants.getWheelFXConfig());
  }

  @Override
  public void setPercent(double pct) {
    logger.info("Setting percent to {}", pct);

    talonFx.set(pct);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFx, true);
  }
}
