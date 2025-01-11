package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ClimbConstants {
  public static int kWheelFxId = 0;
  public static int kArmFxId = 0;

  public static final AngularVelocity kWheelCloseEnough = DegreesPerSecond.of(5.0);
  public static final Angle kArmCloseEnough = Degrees.of(5.0);
  public static final Angle kArmMaxFwd = Degrees.of(100);
  public static final Angle kArmMaxRev = Degrees.of(-100);
  public static final Angle kArmZeroTicks = Degrees.of(1530);

  public static TalonFXConfiguration getWheelFXConfig() {
    TalonFXConfiguration wheelFxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(10)
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(8)
            .withSupplyCurrentLowerTime(0.02)
            .withSupplyCurrentLimitEnable(true);
    wheelFxConfig.CurrentLimits = current;

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
    wheelFxConfig.HardwareLimitSwitch = hwLimit;

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
    wheelFxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);
    wheelFxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Coast);
    wheelFxConfig.MotorOutput = motorOut;

    return wheelFxConfig;
  }

  public static TalonFXConfiguration getArmFxConfig() {
    TalonFXConfiguration armFxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(10)
            .withStatorCurrentLimitEnable(false)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(10)
            .withSupplyCurrentLowerLimit(8)
            .withSupplyCurrentLowerTime(0.02)
            .withSupplyCurrentLimitEnable(true);
    armFxConfig.CurrentLimits = current;

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
    armFxConfig.HardwareLimitSwitch = hwLimit;

    SoftwareLimitSwitchConfigs swLimit =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kArmMaxFwd)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kArmMaxRev);
    armFxConfig.SoftwareLimitSwitch = swLimit;

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
    armFxConfig.Slot0 = slot0;

    MotionMagicConfigs motionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kA(0)
            .withMotionMagicExpo_kV(0)
            .withMotionMagicJerk(0);
    armFxConfig.MotionMagic = motionMagic;

    MotorOutputConfigs motorOut =
        new MotorOutputConfigs()
            .withDutyCycleNeutralDeadband(0.01)
            .withNeutralMode(NeutralModeValue.Coast);
    armFxConfig.MotorOutput = motorOut;

    return armFxConfig;
  }
}
