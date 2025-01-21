package frc.robot.subsystems.biscuit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.BiscuitConstants;
import frc.robot.subsystems.biscuit.BiscuitIO.BiscuitIOInputs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class BiscuitIOFX implements BiscuitIO {

  private Logger logger;
  private TalonFX talon;

  private final Angle sensorInitial;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<ForwardLimitTypeValue> fwdLimitSwitch;
  private boolean didZero;
  private boolean fwdLimitSwitchOpen;
  private Angle offset;
  private Alert rangeAlert = new Alert("Biscuit overextended! Shuting down!", AlertType.kError);

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
    configurator.apply(BiscuitConstants.getFXConfig());
    // Set our variables
    velocity = talon.getVelocity();
    position = talon.getPosition();
  }

  public void setPosition(Angle position) {
    talon.setControl(positionRequest.withPosition(position));
  }

  public void updateInputs(BiscuitIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, position, fwdLimitSwitch);
    inputs.velocity = velocity.getValue();
    inputs.position = position.getValue();
    inputs.fwdLimitSwitchOpen = fwdLimitSwitch.getValueAsDouble() == 1;
    inputs.didZero = didZero;
  }

  public void registerWith(TelemetryService telemetry) {
    telemetry.register(talon, true);
  }

  public void zero() {
    didZero = false;
    if (fwdLimitSwitchOpen == true) {
      Angle pos = position.getValue();
      offset = BiscuitConstants.kZero.minus(pos);
      didZero = true;
    } else {
      rangeAlert.set(true);
      logger.error("Biscuit overextended! Shutting down movement!");
      configurator.apply(BiscuitConstants.disableTalon());
    }
  }
}
