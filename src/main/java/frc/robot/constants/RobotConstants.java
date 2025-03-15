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
import org.slf4j.LoggerFactory;

public class RobotConstants {
  private org.slf4j.Logger logger = LoggerFactory.getLogger(RobotConstants.class);
  public static final String protoSerial = "032243F2";
  public static final boolean isComp = !RobotController.getSerialNumber().equals(protoSerial);
  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kJoystickDeadband = 0.1;
  public static final double kTriggerDeadband = 0.5;
  public static final double kTestingDeadband = 0.5;

  // Elevator
  public static Angle kElevatorFunnelSetpoint; // Elevator
  public static Angle kElevatorStowSetpoint; // Elevator

  // Biscuit
  public static TalonFXSConfiguration talonFXSConfig;
  public static MotionMagicConfigs alageMotionConfig;
  public static MotionMagicConfigs noAlageMotionConfig;

  public static double kTicksPerRot;

  public static double kBiscuitZero;
  public static double kSafeToStowUpper;
  public static double kSafeToStowLower;

  // Speeds
  // public static final double kDosntHaveAlgaeSpeed = 500;

  // Setpoints
  // Idle
  public static Angle kStowSetpoint;
  public static Angle kFunnelSetpoint;
  public static Angle kPrestageSetpoint;
  public static Angle kPrestageAlgaeSetpoint;

  // Algae removal
  public static Angle kL2AlgaeSetpoint;
  public static Angle kL3AlgaeSetpoint;

  public static Angle kL2AlgaeRemovalSetpoint;
  public static Angle kL3AlgaeRemovalSetpoint;

  // Coral score
  public static Angle kL1CoralSetpoint;
  public static Angle kL2CoralSetpoint;
  public static Angle kL3CoralSetpoint;
  public static Angle kL4CoralSetpoint;

  // Algae obtaining
  public static Angle kFloorAlgaeSetpoint;
  public static Angle kMicAlgaeSetpoint;
  public static Angle kHpAlgaeSetpoint;

  // Algae scoring
  public static Angle kProcessorSetpoint;
  public static Angle kBargeSetpoint;
  // public static Angle kBargeBackwardSetpoint;

  public static double kTagAlignThreshold;

  public static final int kMinAutoSwitchID = 4;
  public static final int kMaxAutoSwitchID = 9;

  public RobotConstants() {
    if (isComp) {
      // Comp bot constants
      kElevatorFunnelSetpoint = CompConstants.kElevatorFunnelSetpoint;
      kElevatorStowSetpoint = CompConstants.kElevatorStowSetpoint;
      talonFXSConfig = CompConstants.getFXSConfig();
      alageMotionConfig = CompConstants.getAlgaeMotionConfig();
      noAlageMotionConfig = CompConstants.getNoAlgaeMotionConfig();
      kTicksPerRot = 80;
      logger.info("Using Comp Constants");

      // Biscuit

      kBiscuitZero = CompConstants.kZero;
      kSafeToStowUpper = CompConstants.kSafeToStowUpper;
      kSafeToStowLower = CompConstants.kSafeToStowLower;

      // Setpoints
      // Idle
      kStowSetpoint = CompConstants.kBiscuitStowSetpoint;
      kFunnelSetpoint = CompConstants.kFunnelSetpoint;
      kPrestageSetpoint = CompConstants.kPrestageSetpoint;
      kPrestageAlgaeSetpoint = CompConstants.kBiscuitStowSetpoint;

      // Algae removal
      kL2AlgaeSetpoint = CompConstants.kL2AlgaeSetpoint;
      kL3AlgaeSetpoint = CompConstants.kL3AlgaeSetpoint;

      kL2AlgaeRemovalSetpoint = CompConstants.kL2AlgaeRemovalSetpoint;
      kL3AlgaeRemovalSetpoint = CompConstants.kL3AlgaeRemovalSetpoint;

      // Coral score
      kL1CoralSetpoint = CompConstants.kL1CoralSetpoint;
      kL2CoralSetpoint = CompConstants.kL2CoralSetpoint;
      kL3CoralSetpoint = CompConstants.kL3CoralSetpoint;
      kL4CoralSetpoint = CompConstants.kL4CoralSetpoint;

      // Algae obtaining
      kFloorAlgaeSetpoint = CompConstants.kFloorAlgaeSetpoint;
      kMicAlgaeSetpoint = CompConstants.kMicAlgaeSetpoint;
      kHpAlgaeSetpoint = CompConstants.kHpAlgaeSetpoint;

      // Algae scoring
      kProcessorSetpoint = CompConstants.kProcessorSetpoint;
      kBargeSetpoint = CompConstants.kBargeSetpoint;
      // kBargeBackwardSetpoint = CompConstants.kBargeBackwardSetpoint;
      kTagAlignThreshold = CompConstants.kTagAlignThreshold;
    } else {
      // Proto constants
      kElevatorFunnelSetpoint = ProtoConstants.kElevatorFunnelSetpoint;
      kElevatorStowSetpoint = ProtoConstants.kElevatorStowSetpoint;
      talonFXSConfig = ProtoConstants.getFXSConfig();
      alageMotionConfig = ProtoConstants.getAlgaeMotionConfig();
      noAlageMotionConfig = ProtoConstants.getNoAlgaeMotionConfig();
      kTicksPerRot = 160;
      logger.info("Using Proto Constants");

      // Biscuit

      kBiscuitZero = ProtoConstants.kZero;
      kSafeToStowUpper = ProtoConstants.kSafeToStowUpper;
      kSafeToStowLower = ProtoConstants.kSafeToStowLower;

      // Setpoints
      // Idle
      kStowSetpoint = ProtoConstants.kBiscuitStowSetpoint;
      kFunnelSetpoint = ProtoConstants.kFunnelSetpoint;
      kPrestageSetpoint = ProtoConstants.kPrestageSetpoint;
      kPrestageAlgaeSetpoint = ProtoConstants.kPrestageAlgaeSetpoint;

      // Algae removal
      kL2AlgaeSetpoint = ProtoConstants.kL2AlgaeSetpoint;
      kL3AlgaeSetpoint = ProtoConstants.kL3AlgaeSetpoint;

      kL2AlgaeRemovalSetpoint = ProtoConstants.kL2AlgaeRemovalSetpoint;
      kL3AlgaeRemovalSetpoint = ProtoConstants.kL3AlgaeRemovalSetpoint;

      // Coral score
      kL1CoralSetpoint = ProtoConstants.kL1CoralSetpoint;
      kL2CoralSetpoint = ProtoConstants.kL2CoralSetpoint;
      kL3CoralSetpoint = ProtoConstants.kL3CoralSetpoint;
      kL4CoralSetpoint = ProtoConstants.kL4CoralSetpoint;

      // Algae obtaining
      kFloorAlgaeSetpoint = ProtoConstants.kFloorAlgaeSetpoint;
      kMicAlgaeSetpoint = ProtoConstants.kMicAlgaeSetpoint;
      kHpAlgaeSetpoint = ProtoConstants.kHpAlgaeSetpoint;

      // Algae scoring
      kProcessorSetpoint = ProtoConstants.kProcessorSetpoint;
      kBargeSetpoint = ProtoConstants.kBargeSetpoint;
      // kBargeBackwardSetpoint = ProtoConstants.kBargeBackwardSetpoint;

      kTagAlignThreshold = ProtoConstants.kTagAlignThreshold;
    }
  }

  public static class ProtoConstants {

    // Biscuit
    public static TalonFXSConfiguration talonFXSConfig;
    public static MotionMagicConfigs alageMotionConfig;
    public static MotionMagicConfigs noAlageMotionConfig;

    public static double kTicksPerRot = 160;

    public static final double kZero = .37;
    public static final double kSafeToStowUpper = 40;
    public static final double kSafeToStowLower = -5;

    // Speeds
    // public static final double kDosntHaveAlgaeSpeed = 500;

    // Setpoints
    // Idle
    public static Angle kBiscuitStowSetpoint = Rotations.of(1.862);
    public static Angle kFunnelSetpoint = kBiscuitStowSetpoint;
    public static Angle kPrestageSetpoint = kBiscuitStowSetpoint;
    public static Angle kPrestageAlgaeSetpoint = Rotations.of(9.089);

    // Algae removal
    public static Angle kL2AlgaeSetpoint = Rotations.of(24.104);
    public static Angle kL3AlgaeSetpoint = Rotations.of(24.562);

    public static Angle kL2AlgaeRemovalSetpoint = kBiscuitStowSetpoint;
    public static Angle kL3AlgaeRemovalSetpoint = kBiscuitStowSetpoint;

    public static double kTagAlignThreshold = 20.0;

    // Coral score
    public static Angle kL1CoralSetpoint = kBiscuitStowSetpoint;
    public static Angle kL2CoralSetpoint = kBiscuitStowSetpoint;
    public static Angle kL3CoralSetpoint = kBiscuitStowSetpoint;
    public static Angle kL4CoralSetpoint = kBiscuitStowSetpoint;

    // Algae obtaining
    public static Angle kFloorAlgaeSetpoint = Rotations.of(49.627);
    public static Angle kMicAlgaeSetpoint = Rotations.of(51.61872);
    public static Angle kHpAlgaeSetpoint = Rotations.of(16.97559);

    // Algae scoring
    public static Angle kProcessorSetpoint = Rotations.of(41.193);
    public static Angle kBargeSetpoint = Rotations.of(12.3489);
    // public static Angle kBargeBackwardSetpoint = Rotations.of(-12.3489); // 9.089

    // Elevator
    public static Angle kElevatorFunnelSetpoint = Rotations.of(2.03125);
    public static Angle kElevatorStowSetpoint = kElevatorFunnelSetpoint;
    public static Angle kMaxFwd = kMicAlgaeSetpoint.plus(Rotations.of(5));
    public static Angle kMaxRev = kPrestageSetpoint.minus(Rotations.of(5));

    public static MotionMagicConfigs getAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(50)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(600);
      return algaeConfig;
    }

    public static MotionMagicConfigs getNoAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(80)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1800);
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
    public static Angle kElevatorFunnelSetpoint = Rotations.of(0.3676757);
    public static Angle kElevatorStowSetpoint = kElevatorFunnelSetpoint;

    // Biscuit
    public static double kTicksPerRot = 160;

    public static final double kZero = .67;
    public static final double kSafeToStowUpper = 40 / 2;
    public static final double kSafeToStowLower = -5 / 2;

    // Speeds
    // public static final double kDosntHaveAlgaeSpeed = 500;

    // Setpoints
    // Idle
    public static Angle kBiscuitStowSetpoint = Rotations.of(1.862 / 2);
    public static Angle kFunnelSetpoint = kBiscuitStowSetpoint;
    public static Angle kPrestageSetpoint = Rotations.of(-2.94);
    public static Angle kPrestageAlgaeSetpoint = kBiscuitStowSetpoint;

    // Algae removal
    public static Angle kL2AlgaeSetpoint = Rotations.of(7.39);
    public static Angle kL3AlgaeSetpoint = Rotations.of(7.39);

    public static Angle kL2AlgaeRemovalSetpoint = kPrestageSetpoint;
    public static Angle kL3AlgaeRemovalSetpoint = kPrestageSetpoint;

    public static double kTagAlignThreshold = 20.0 / 2;

    // Coral score
    public static Angle kL1CoralSetpoint = kPrestageSetpoint;
    public static Angle kL2CoralSetpoint = kPrestageSetpoint;
    public static Angle kL3CoralSetpoint = kPrestageSetpoint;
    public static Angle kL4CoralSetpoint = kPrestageSetpoint;

    // Algae obtaining
    public static Angle kFloorAlgaeSetpoint = Rotations.of(20.039);
    public static Angle kMicAlgaeSetpoint = Rotations.of(12.87085);
    public static Angle kHpAlgaeSetpoint = kBiscuitStowSetpoint;

    // Algae scoring
    public static Angle kProcessorSetpoint = Rotations.of(15.0139);
    public static Angle kBargeSetpoint = Rotations.of(2.697);
    // public static Angle kBargeBackwardSetpoint = Rotations.of(-12.3489 / 2); // 9.089

    public static Angle kMaxFwd = kFloorAlgaeSetpoint.plus(Rotations.of(5));
    public static Angle kMaxRev = kPrestageSetpoint.minus(Rotations.of(5));

    public static MotionMagicConfigs getAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(50)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(600);
      return algaeConfig;
    }

    public static MotionMagicConfigs getNoAlgaeMotionConfig() {
      MotionMagicConfigs algaeConfig =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(300)
              .withMotionMagicCruiseVelocity(80)
              .withMotionMagicExpo_kA(0)
              .withMotionMagicExpo_kV(0)
              .withMotionMagicJerk(1800);
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
