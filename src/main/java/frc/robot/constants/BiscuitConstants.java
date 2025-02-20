package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;

public class BiscuitConstants {

  public static final double kZero = .36;
  public static final double kTicksPerRot = 160;
  public static final int talonID = 25;
  public static final double kCloseEnough = 0.05;
  public static final Angle kMaxFwd = Rotations.of(51.04735 + 5);
  public static final Angle kMaxRev = Rotations.of(-12.3489 - 5);
  public static final double kSafeToStowUpper = 40;
  public static final double kSafeToStowLower = -5;

  // Setpoints
  // Idle
  public static final Angle kStowSetpoint = Rotations.of(1.862);
  public static final Angle kFunnelSetpoint = kStowSetpoint;
  public static final Angle kPrestageSetpoint = kStowSetpoint;

  // Algae removal
  public static final Angle kL2AlgaeSetpoint = Rotations.of(20.848);
  public static final Angle kL3AlgaeSetpoint = Rotations.of(24.562);

  public static final Angle kL2AlgaeRemovalSetpoint = kL2AlgaeSetpoint;
  public static final Angle kL3AlgaeRemovalSetpoint = kL3AlgaeSetpoint;

  public static final Angle kSafeAlgaeRemovalSetpoint = Rotations.of(0.0);
  public static final Angle kSafeAlgaeRemovalRotateSetpoint = Rotations.of(0.0);

  // Coral score
  public static final Angle kL1CoralSetpoint = kStowSetpoint;
  public static final Angle kL2CoralSetpoint = kStowSetpoint;
  public static final Angle kL3CoralSetpoint = kStowSetpoint;
  public static final Angle kL4CoralSetpoint = kStowSetpoint;

  // Algae obtaining
  public static final Angle kFloorAlgaeSetpoint = Rotations.of(49.627);
  public static final Angle kMicAlgaeSetpoint = Rotations.of(51.61872);
  public static final Angle kHpAlgaeSetpoint = Rotations.of(16.97559);

  // Algae scoring
  public static final Angle kProcessorSetpoint = Rotations.of(41.193);
  public static final Angle kBargeSetpoint = Rotations.of(12.3489);
  public static final Angle kBargeBackwardSetpoint = Rotations.of(-12.3489);

  // jogging
  public static final double kJogAmountUp = 10;
  public static final double kJogAmountDown = -10;

  // Disables the TalonFXS by setting its voltage to zero.
  public static VoltageConfigs disableTalon() {
    VoltageConfigs voltage =
        new VoltageConfigs().withPeakForwardVoltage(0.0).withPeakReverseVoltage(0.0);
    getFXSConfig().Voltage = voltage;
    return voltage;
  }

  public static TalonFXSConfiguration getFXSConfig() {
    TalonFXSConfiguration fxsConfig = new TalonFXSConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(0)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(5)
            .withSupplyCurrentLowerTime(2)
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
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kMaxFwd)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kMaxRev);
    fxsConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0)
            .withKS(0)
            .withKV(0.1)
            .withKA(0);
    fxsConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(500)
            .withMotionMagicCruiseVelocity(100)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(1000);
    fxsConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Brake);
    fxsConfig.MotorOutput = motorOut;

    CommutationConfigs commutation =
        new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST);
    fxsConfig.Commutation = commutation;

    ExternalFeedbackConfigs feedBack =
        new ExternalFeedbackConfigs()
            .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Commutation);
    fxsConfig.ExternalFeedback = feedBack;

    return fxsConfig;
  }
}
