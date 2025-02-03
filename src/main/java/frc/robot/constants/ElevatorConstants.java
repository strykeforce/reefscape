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

  public static final double kCloseEnoughRotations = 0.0083;
  public static final double kMaxFwd = 0; // TODO all of these fields need to be filled out
  public static final double kMaxRev = 0;
  public static final int kZeroMultiple =
      0; // some constant to multiply, add by to turn the analog input into a position
  public static final double kZeroSpeed = -.05;
  public static final int kZeroCounter = 3;
  public static final double kZeroedThreshhold = .0001;

  public static final int heightAnalogID = 0;
  public static final int kFxIDMain = 20;
  public static final int kFxIDFollow = 21;

  public static final double kJogAmount = 0.1;

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

  public static TalonFXConfiguration getBothFXConfig() {
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(8)
            .withSupplyCurrentLowerTime(0.02)
            .withSupplyCurrentLimitEnable(true);
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
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(kMaxFwd)
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(kMaxRev);
    fxConfig.SoftwareLimitSwitch = swLimit;

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(0.4)
            .withKI(0.1)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0)
            .withKS(0)
            .withKV(0.12)
            .withKA(0);
    fxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(130)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(1000);
    fxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Coast)
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
}
