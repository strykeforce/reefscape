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
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

public class RobotConstants {
  private Logger logger;

  public static final String protoSerial = "032243F2";
  public static final boolean isComp = !RobotController.getSerialNumber().equals(protoSerial);
  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kJoystickDeadband = 0.1;
  public static final double kTriggerDeadband = 0.5;
  public static final double kTestingDeadband = 0.5;

  public static double kZero;
  public static Angle kFunnelSetpoint;
  public static Angle kStowSetpoint;
  public static TalonFXSConfiguration talonFXSConfig;
  public static MotionMagicConfigs alageMotionConfig;
  public static MotionMagicConfigs noAlageMotionConfig;
  // Algae obtaining
  public static final Angle kFloorAlgaeSetpoint = Rotations.of(49.627);
  public static final Angle kMicAlgaeSetpoint = Rotations.of(51.61872);
  public static final Angle kHpAlgaeSetpoint = Rotations.of(16.97559);

  // Algae scoring
  public static final Angle kProcessorSetpoint = Rotations.of(41.193);
  public static final Angle kBargeSetpoint = Rotations.of(12.3489);
  public static final Angle kBargeBackwardSetpoint = Rotations.of(-12.3489);

  public RobotConstants() {
    logger.recordOutput("RobotConstants/Using Comp Constants", isComp);
    if (isComp) {
      // Comp bot constants
      kZero = CompConstants.kZero;
      kFunnelSetpoint = CompConstants.kFunnelSetpoint;
      kStowSetpoint = CompConstants.kStowSetpoint;
      talonFXSConfig = CompConstants.getFXSConfig();
      alageMotionConfig = CompConstants.getAlgaeMotionConfig();
      noAlageMotionConfig = CompConstants.getNoAlgaeMotionConfig();
    } else {
      // Proto constants
      kZero = ProtoConstants.kZero;
      kFunnelSetpoint = ProtoConstants.kFunnelSetpoint;
      kStowSetpoint = ProtoConstants.kStowSetpoint;
      talonFXSConfig = ProtoConstants.getFXSConfig();
      alageMotionConfig = ProtoConstants.getAlgaeMotionConfig();
      noAlageMotionConfig = ProtoConstants.getNoAlgaeMotionConfig();
    }
  }

  public static class ProtoConstants {
    public static final double kZero = 36;
    public static final Angle kFunnelSetpoint = Rotations.of(2.03125);
    public static final Angle kStowSetpoint = kFunnelSetpoint;
    public static final Angle kMaxFwd = kMicAlgaeSetpoint.plus(Rotations.of(5));
    public static final Angle kMaxRev = kBargeBackwardSetpoint.minus(Rotations.of(5));

    public static MotionMagicConfigs getAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(500)
              .withMotionMagicCruiseVelocity(100)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1000);
      return algaeConfig;
    }

    public static MotionMagicConfigs getNoAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(500)
              .withMotionMagicCruiseVelocity(100)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1000);
      return algaeConfig;
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

      fxsConfig.MotionMagic = getNoAlgaeMotionConfig();

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

  public static class CompConstants {
    public static final double kZero = .37;
    public static final Angle kFunnelSetpoint = Rotations.of(0.3676757);
    public static final Angle kStowSetpoint = kFunnelSetpoint;
    public static final Angle kMaxFwd = kMicAlgaeSetpoint.plus(Rotations.of(5));
    public static final Angle kMaxRev = kBargeBackwardSetpoint.minus(Rotations.of(5));

    public static MotionMagicConfigs getAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(500)
              .withMotionMagicCruiseVelocity(100)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1000);
      return algaeConfig;
    }

    public static MotionMagicConfigs getNoAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(500)
              .withMotionMagicCruiseVelocity(100)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1000);
      return algaeConfig;
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

      fxsConfig.MotionMagic = getNoAlgaeMotionConfig();

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
}
