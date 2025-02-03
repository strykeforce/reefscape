package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;

public class BiscuitConstants {
  // These are all wrong right now because we don't have any actual info

  public static Angle kZero = Rotations.of(42); // Will need to be experimentally determined
  public static int talonID = 3;
  public static double kCloseEnough = 2137473647; // This is a little out of wack.
  public static final Angle kMaxFwd = Rotations.of(100);
  public static final Angle kMaxRev = Rotations.of(-100);

  // Setpoints
  // Idle
  public static final Angle kStowSetpoint = Rotations.of(0.0);
  public static final Angle kFunnelSetpoint = Rotations.of(0.0);
  public static final Angle kPrestageSetpoint = Rotations.of(0.0);

  // Algae removal
  public static final Angle kL2AlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kL3AlgaeSetpoint = Rotations.of(0.0);

  public static final Angle kL2AlgaeRemovalSetpoint = Rotations.of(0.0);
  public static final Angle kL3AlgaeRemovalSetpoint = Rotations.of(0.0);

  public static final Angle kSafeAlgaeRemovalSetpoint = Rotations.of(0.0);
  public static final Angle kSafeAlgaeRemovalRotateSetpoint = Rotations.of(0.0);

  // Coral score
  public static final Angle kL1CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL2CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL3CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL4CoralSetpoint = Rotations.of(0.0);

  // Algae obtaining
  public static final Angle kFloorAlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kMicAlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kHpAlgaeSetpoint = Rotations.of(0.0);

  // Algae scoring
  public static final Angle kProcessorSetpoint = Rotations.of(0.0);
  public static final Angle kBargeSetpoint = Rotations.of(0.0);

  // Disables the TalonFXS by setting it's voltage to zero. Not very shocking.
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
            .withStatorCurrentLimit(10)
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(8)
            .withSupplyCurrentLowerTime(0.02)
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
            .withNeutralMode(NeutralModeValue.Coast);
    fxsConfig.MotorOutput = motorOut;

    return fxsConfig;
  }
}
