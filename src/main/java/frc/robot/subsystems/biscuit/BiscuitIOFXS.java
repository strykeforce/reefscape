package frc.robot.subsystems.biscuit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.biscuit.BiscuitIO.BiscuitIOInputs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class BiscuitIOFXS implements BiscuitIO {

  private Logger logger;
  private TalonFXS talon;

  private final Angle sensorInitial;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<ForwardLimitTypeValue> fwdLimitSwitch;
  private StatusSignal<Angle> rawQuadrature;
  private StatusSignal<Angle> rawPulseWidth;
  private boolean didZero;
  private boolean fwdLimitSwitchOpen;
  private Angle offset;
  private Alert rangeAlert = new Alert("Biscuit overextended! Shuting down!", AlertType.kError);
  private Boolean lastHadAlgae = false;

  TalonFXSConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0);

  public BiscuitIOFXS() {
    // Logger initialization with class name
    logger = LoggerFactory.getLogger(this.getClass());
    // Moter initialization with ID from constants
    talon = new TalonFXS(BiscuitConstants.talonID);
    // Set the starting encoder position
    sensorInitial = talon.getPosition().getValue();

    // Reset and configure motor settings
    configurator = talon.getConfigurator();
    configurator.apply(new TalonFXSConfiguration());
    configurator.apply(RobotConstants.talonFXSConfig);

    // Set our variables
    velocity = talon.getVelocity();
    position = talon.getPosition();
    rawQuadrature = talon.getRawQuadraturePosition();
    rawQuadrature.setUpdateFrequency(20);
    rawPulseWidth = talon.getRawPulseWidthPosition();
    rawPulseWidth.setUpdateFrequency(200);
    zero();
  }

  @Override
  public void setPosition(Angle position, boolean hasAlgae) {
    if (hasAlgae != lastHadAlgae) {
      if (hasAlgae) {
        configurator.apply(RobotConstants.alageMotionConfig);
      } else {
        configurator.apply(RobotConstants.noAlageMotionConfig);
      }
      lastHadAlgae = hasAlgae;
    }
    talon.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void updateInputs(BiscuitIOInputs inputs) {
    inputs.velocity = velocity.getValueAsDouble();
    inputs.position = position.getValueAsDouble();
    inputs.rawPulseWidth = rawPulseWidth.getValueAsDouble();
    inputs.didZero = didZero;

    BaseStatusSignal.refreshAll(velocity, position, rawPulseWidth);
  }

  @Override
  public void registerWith(TelemetryService telemetry) {
    telemetry.register(talon, true);
  }

  @Override
  public boolean zero() {
    didZero = false;
    double pos = MathUtil.inputModulus(rawPulseWidth.refresh().getValueAsDouble(), 0, 1);
    double pos2 = MathUtil.inputModulus(rawPulseWidth.refresh().getValueAsDouble(), 0, 1);
    double pos3 = MathUtil.inputModulus(rawPulseWidth.refresh().getValueAsDouble(), 0, 1);
    if (pos3 != 1.0) {
      double setPos = RobotConstants.kTicksPerRot * (RobotConstants.kBiscuitZero - pos3);
      talon.setPosition(setPos);
      logger.info(
          "set Biscuit position to "
              + setPos
              + ", Abs Pos 1: "
              + pos
              + ", Abs Pos 2: "
              + pos2
              + "Abs Pos 3: "
              + pos3);
      didZero = true;
    }
    return didZero;
  }
}
