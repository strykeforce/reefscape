package frc.robot.subsystems.example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ExampleConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ExampleIOFX implements ExampleIO {
  // private objects
  private Logger logger;
  private TalonFX talonFx;

  // storage variables
  private final Angle absSensorInitial;
  private Angle relSetpointOffset;
  private Angle setpoint;

  // FX Access objects
  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Angle> curPosition;
  StatusSignal<AngularVelocity> curVelocity;

  public ExampleIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(ExampleConstants.kExampleFxId);
    absSensorInitial =
        talonFx.getPosition().getValue(); // relative encoder starts up as absolute position offset

    // Config controller
    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(ExampleConstants.getFXConfig());

    // Attach status signals
    curPosition = talonFx.getPosition();
    curVelocity = talonFx.getVelocity();
  }

  @Override
  public void zero() {
    relSetpointOffset = ExampleConstants.kZeroTicks;
    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}",
        absSensorInitial,
        ExampleConstants.kZeroTicks,
        absSensorInitial.minus(ExampleConstants.kZeroTicks));
  }

  @Override
  public void setPosition(Angle position) {
    setpoint = position.plus(relSetpointOffset);
    talonFx.setControl(positionRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {
    BaseStatusSignal.refreshAll(curVelocity, curPosition);
    inputs.velocity = curVelocity.getValue();
    inputs.position = curPosition.getValue().minus(relSetpointOffset);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFx, true);
  }
}
