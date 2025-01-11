package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ExampleConstants;

public class ClimbArmIOFX implements ClimbArmIO {
  private Logger logger;
  private TalonFX talonFx;
  
  private final Angle absSensorInitial;
  private Angle relSetpointOffset;
  private Angle setpoint;

  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest = new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Angle> currPosition;

  public ClimbArmIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(ClimbConstants.kArmFxId);
    absSensorInitial =
        talonFx.getPosition().getValue(); // relative encoder starts up as absolute position offset

    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(ClimbConstants.getArmFxConfig());

    currPosition = talonFx.getPosition();
  }

  @Override
  public void setPosition(Angle position) {
    setpoint = position.plus(relSetpointOffset);

    logger.info("Setting position to {} degrees", setpoint.in(Degrees));

    talonFx.setControl(positionRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(ClimbArmIOInputs inputs) {
      inputs.position = currPosition.refresh().getValue();
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
}
