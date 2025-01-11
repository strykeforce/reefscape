package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ExampleConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ClimbArmIOFX implements ClimbArmIO {
  private Logger logger;
  private TalonFX talonFx;

  private final Angle absSensorInitial;
  private Angle relSetpointOffset;
  private Angle setpoint;

  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Angle> curPosition;

  public ClimbArmIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(ClimbConstants.kArmFxId);
    absSensorInitial =
        talonFx.getPosition().getValue(); // relative encoder starts up as absolute position offset

    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(ClimbConstants.getArmFxConfig());

    curPosition = talonFx.getPosition();
  }

  @Override
  public void setPosition(Angle position) {
    setpoint = position.plus(relSetpointOffset);

    logger.info("Setting position to {} rotations", setpoint.in(Rotations));

    talonFx.setControl(positionRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(ClimbArmIOInputs inputs) {
    inputs.position = curPosition.refresh().getValue();
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
