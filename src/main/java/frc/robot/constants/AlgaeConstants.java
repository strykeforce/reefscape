package frc.robot.constants;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class AlgaeConstants {
  public static final int kFxId = 30;

  public static final double kCloseEnough = 0.1;
  public static final double kMaxFwd = 100;
  public static final double kMaxRev = -100;

  public static final double kHoldSpeed = 1;
  public static final double kCoralHoldSpeed = -0.05;
  public static final double kBargeScoreSpeed = -1;
  public static final double kProcessorScoreSpeed = -0.5; // was -1.0
  public static final double kCoralScoreSpeed = 0.3; // 0.5; was 0.4
  public static final double kIntakingSpeed = 1; // 0.75
  public static final double kCoralIntakingSpeed = -0.3; // -0.75;

  public static final double kHasAlgaeVelThreshold = 70; // 10
  public static final double kSuperCycleHasAlgaeVelThres = 40; // was 40
  public static final int kHasAlgaeCounts = 2;

  public static final double kHasCoralVelThreshold = 30;
  public static final int kHasCoralCounts = 3;

  public static final double kCoralScoringTime = 1; // FIXME

  // Example Talon FX Config
  public static TalonFXSConfiguration getFXConfig() {
    TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(25) // 40
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(10) // 2
            .withSupplyCurrentLowerTime(1)
            .withSupplyCurrentLimitEnable(true);
    fxsConfig.CurrentLimits = current;

    HardwareLimitSwitchConfigs hwLimit =
        new HardwareLimitSwitchConfigs()
            .withForwardLimitAutosetPositionEnable(false)
            .withForwardLimitEnable(false)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
            .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
            .withReverseLimitAutosetPositionEnable(false)
            .withReverseLimitEnable(false)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin);
    fxsConfig.HardwareLimitSwitch = hwLimit;

    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false);
    fxsConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0);
    fxsConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);
    fxsConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    fxsConfig.MotorOutput = motorOut;

    CommutationConfigs commutationConfigs =
        new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST);
    fxsConfig.Commutation = commutationConfigs;

    ExternalFeedbackConfigs externalFeedbackConfigs =
        new ExternalFeedbackConfigs()
            .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Commutation);
    fxsConfig.ExternalFeedback = externalFeedbackConfigs;

    return fxsConfig;
  }
}
