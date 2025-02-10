package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.ElevatorConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ElevatorIOFX implements ElevatorIO {
  private Logger logger;
  private TalonFX talonFxFront;
  private TalonFX talonFxBack;

  private Angle setpoints;

  // FX Access objects
  TalonFXConfigurator configuratorFront;
  TalonFXConfigurator configuratorBack;
  StatusSignal<Angle> currPosition;
  StatusSignal<AngularVelocity> currVelocity;
  public AnalogInput heightAnalogInput = new AnalogInput(ElevatorConstants.heightAnalogID);
  private MotionMagicVoltage positionRequestMain =
      new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);
  private Follower positionRequestFollow = new Follower(ElevatorConstants.kFxIDMain, false);
  private DutyCycleOut openLoopVelocityRequest = new DutyCycleOut(0);
  private VoltageOut openLoopVoltageRequest = new VoltageOut(0);

  public ElevatorIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFxFront = new TalonFX(ElevatorConstants.kFxIDMain);
    talonFxBack = new TalonFX(ElevatorConstants.kFxIDFollow);

    // controller config
    configuratorFront = talonFxFront.getConfigurator();
    configuratorBack = talonFxBack.getConfigurator();
    configuratorFront.apply(ElevatorConstants.getBothFXConfig());
    configuratorBack.apply(ElevatorConstants.getBothFXConfig());

    // Attach status signals
    currPosition = talonFxFront.getPosition();
    currVelocity = talonFxFront.getVelocity();

    talonFxBack.setControl(positionRequestFollow);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(currVelocity, currPosition);
    inputs.velocity = currVelocity.getValueAsDouble();
    inputs.position = currPosition.getValueAsDouble();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFxFront, true);
    telemetryService.register(talonFxBack, true);
  }

  @Override
  public void setPosition(Angle position) {
    talonFxFront.setControl(positionRequestMain.withPosition(position));

    setpoints = position;
  }

  @Override
  public void setVelocityOpenLoop(double dutyCycleOut) {
    talonFxFront.setControl(openLoopVelocityRequest.withOutput(dutyCycleOut));
  }

  @Override
  public void setVoltageOpenLoop(double voltsOut) {
    talonFxFront.setControl(openLoopVoltageRequest.withOutput(voltsOut));
  }

  @Override
  public void setCurrentLimitConfig(CurrentLimitsConfigs config) {
    configuratorFront.apply(config);
    configuratorBack.apply(config);
  }

  @Override
  public void setSoftLimitConfig(SoftwareLimitSwitchConfigs config) {
    configuratorFront.apply(config);
    configuratorBack.apply(config);
  }

  @Override
  public void zero() {
    talonFxFront.setPosition(0.0);
    talonFxBack.setPosition(0.0);
    setVelocityOpenLoop(0.0);
  }
}
