package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ClimbIOServoFX implements ClimbIO {
  // will prob need healthchecks eventually
  private Logger logger;
  private TalonFX talonFxPivotArmFront;
  private TalonFX talonFxPivotArmBack;
  private Servo ratchetServo;
  private Servo pinServo;

  private final Angle absPivotArmFrontSensorInitial;
  private Angle relSetpointOffset;
  private Angle pivotArmSetpoint;

  TalonFXConfigurator configuratorFront;
  TalonFXConfigurator configuratorBack;
  private MotionMagicDutyCycle positionRequestMain =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);

  StatusSignal<Angle> currPosition;

  public ClimbIOServoFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFxPivotArmFront = new TalonFX(ClimbConstants.kPivotArmFrontFxId);
    ratchetServo = new Servo(ClimbConstants.kRatchetServoId);
    pinServo = new Servo(ClimbConstants.kDeployServoId);

    absPivotArmFrontSensorInitial =
        talonFxPivotArmFront.getPosition().getValue(); // only for logging
    configuratorFront = talonFxPivotArmFront.getConfigurator();
    configuratorBack = talonFxPivotArmBack.getConfigurator();
    configuratorFront.apply(new TalonFXConfiguration());
    configuratorFront.apply(ClimbConstants.getPivotArmFxConfig());
    configuratorBack.apply(new TalonFXConfiguration());
    configuratorBack.apply(ClimbConstants.getPivotArmFxConfig());

    currPosition = talonFxPivotArmFront.getPosition();
  }

  // @Override
  public String getName() {
    return "Climb";
  }

  @Override
  public void setPosition(Angle position) {
    pivotArmSetpoint = position.plus(relSetpointOffset);

    logger.info("Setting position to {} rotations", pivotArmSetpoint.in(Rotations));

    talonFxPivotArmFront.setControl(positionRequestMain.withPosition(pivotArmSetpoint));
  }

  @Override
  public void setRatchetServoPosition(double position) {
    // ratchetServo.setPosition(position);
    ratchetServo.set(position);
    // could be wrong but I'm assuming set rather than setPosition
  }

  @Override
  public void setPinServoPosition(double position) {
    // pinServo.setPosition(position);
    pinServo.set(position);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.position = currPosition.refresh().getValue();
    inputs.ratchetServoPosition = ratchetServo.getPosition();
    inputs.pinServoPosition = pinServo.getPosition();
  }

  @Override
  public void zero() {
    relSetpointOffset = ClimbConstants.kArmZeroTicks;
    /*
    logger.info(
        "Abs: {}, Zero Pos: {}, Offset: {}"
        absPivotArmFrontSensorInitial,
        ClimbConstants.kZeroTicks,
        absPivotArmFrontSensorInitial.minus(ClimbConstants.kZeroTicks));
        setPosition(0.0);

    );
    */
    talonFxPivotArmBack.setPosition(0.0);
    talonFxPivotArmFront.setPosition(0.0);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enable) {
    configuratorFront.apply(
        ClimbConstants.getPivotArmFxConfig()
            .SoftwareLimitSwitch
            .withForwardSoftLimitEnable(enable)
            .withReverseSoftLimitEnable(enable)); // fixme
    // configuratorBack.apply(ClimbConstants.getPivotArmFxConfig().SoftwareLimitSwitch.withForwardSoftLimitEnable(enable)
    // .withReverseSoftLimitEnable());
  }

  @Override
  public void setCurrentLimit(CurrentLimitsConfigs config) {
    configuratorFront.apply(config);
    configuratorBack.apply(config);
  }

  public void goToZero() {}
}
