package frc.robot.subsystems.biscuit;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.BiscuitConstants;
import frc.robot.subsystems.biscuit.BiscuitIO.BiscuitIOInputs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
// import com.ctre.phoenix6.BaseStatusSignal.refreshAll;

public class BiscuitIOFX implements BiscuitIO {

  private Logger logger;
  private TalonFX talon;

  private final Angle sensorInitial;
  private Angle setPoint;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;

  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0);

  public BiscuitIOFX() {
    // Logger initialization with class name
    logger = LoggerFactory.getLogger(this.getClass());
    // Moter initialization with ID from constants
    talon = new TalonFX(BiscuitConstants.talonID);
    // Set the starting encoder position
    sensorInitial = talon.getPosition().getValue();

    // Reset and configure moter settings
    configurator = talon.getConfigurator();
    configurator.apply(new TalonFXConfiguration());
    configurator.apply(BiscuitConstants.talonConfiguration());

    velocity = talon.getVelocity();
    position = talon.getPosition();
  }

  public void setPosition(Angle position) {
    setPoint = position.plus(BiscuitConstants.kZero);
    talon.setControl(positionRequest.withPosition(setPoint));
  }

  public void updateInputs(BiscuitIOInputs inputs) {
    // inputs.velocity = velocity.refresh().getValue();
    inputs.position = position.refresh().getValue().minus(BiscuitConstants.kZero);
  }

  public void registerWith(TelemetryService telemetry) {
    telemetry.register(talon, true);
  }
}
