package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;

public class ElevatorConstants {

  public static final double kCloseEnoughRotations = 0.1;
  public static final double kMaxFwd = 53;
  public static final double kMaxRev = 2;
  public static final int kZeroMultiple =
      0; // some constant to multiply, add by to turn the analog input into a position
  public static final double kZeroSpeed = -.05;
  public static final double kZeroVolts = -0.5;
  public static final int kZeroCounter = 2;
  public static final double kZeroedThreshhold = .025;

  public static final int heightAnalogID = 0;
  public static final int kFxIDMain = 20;
  public static final int kFxIDFollow = 21;

  public static final double kJogAmountUp = 1;
  public static final double kJogAmountDown = -1.5;

  // Setpoints
  // Idle
  public static final Angle kFunnelSetpoint = Rotations.of(2.03125); // was 2.40430
  public static final Angle kStowSetpoint = kFunnelSetpoint;

  // Algae removal
  public static final Angle kL2AlgaeSetpoint = Rotations.of(2.8677);
  public static final Angle kL3AlgaeSetpoint = Rotations.of(2.8677);

  public static final Angle kL2AlgaeRemovalSetpoint = Rotations.of(2.8677);
  public static final Angle kL3AlgaeRemovalSetpoint = Rotations.of(2.8677);

  public static final Angle kSafeAlgaeRemovalSetpoint = Rotations.of(2.8677);
  public static final Angle kSafeAlgaeRemovalRotateSetpoint = Rotations.of(2.8677);

  // Coral score
  public static final Angle kL1CoralSetpoint = Rotations.of(13.04053);
  public static final Angle kL2CoralSetpoint = Rotations.of(21.0786); // 19.62793 -> 21.0786
  public static final Angle kL3CoralSetpoint =
      Rotations.of(31.0901); // was 30.42969 -> 31.7505 -> 31.0901
  public static final Angle kL4CoralSetpoint = Rotations.of(48.28076);

  public static final Angle kPrestageSetpoint = kL3CoralSetpoint;

  // Algae obtaining
  public static final Angle kFloorAlgaeSetpoint = Rotations.of(8.81836);
  public static final Angle kMicAlgaeSetpoint = Rotations.of(17.708);
  public static final Angle kHpAlgaeSetpoint = Rotations.of(14.9063);

  // Algae scoring
  public static final Angle kProcessorSetpoint = Rotations.of(14.9063);
  public static final Angle kBargeSetpoint = Rotations.of(40.913);

  public static TalonFXConfiguration getBothFXConfig() {
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            // .withStatorCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(10)
            .withSupplyCurrentLowerTime(2);
    fxConfig.CurrentLimits = current;

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
    fxConfig.HardwareLimitSwitch = hwLimit;

    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kMaxFwd)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kMaxRev);
    fxConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.36)
            .withKS(0)
            .withKV(0.13)
            .withKA(0);
    fxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs().withMotionMagicCruiseVelocity(50).withMotionMagicAcceleration(100);
    fxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    fxConfig.MotorOutput = motorOut;

    return fxConfig;
  }

  public static CurrentLimitsConfigs getZeroingCurrentLimitsConfigs() {
    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs() // TODO actually have correct limits for zeroing
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(8)
            .withSupplyCurrentLowerTime(0.02)
            .withSupplyCurrentLimitEnable(true);
    return current;
  }

  public static SoftwareLimitSwitchConfigs getZeroingSoftLimitConfigs() {
    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false);

    return swLimit;
  }
}
