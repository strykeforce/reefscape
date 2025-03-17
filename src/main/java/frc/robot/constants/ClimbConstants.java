package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;

public class ClimbConstants {
  public static int kPivotArmFrontFxId = 45;
  // public static int kPivotArmFollowFxId = 46;
  public static int kCANcoderId = 46;
  public static int kDeployServoId = 1;
  public static int kRatchetServoId = 2;
  public static int kCageAlignedDIOId = 11;

  public static final double kPivotArmCloseEnough = 0.01; // fixme
  public static final double kArmMaxFwd = 0.01;
  public static final double kArmMaxRev = -0.29;
  public static final Angle kArmZeroTicks = Degrees.of(1530);

  // Deploy Servo
  public static final double kPinDeployedPosition = 0.75;
  public static final double kPinRetractedPosition = 0.18;

  // Ratchet Servo
  public static final double kRatchetEngagedPos = 0.0;
  public static final double kRatchetDisengagedPos = 1.0;
  
  // Climb positions
  // public static final Angle kClimbCagePos = Rotations.of(); // will test
  public static final double kClimbRatchedEngage = -0.12;
  public static final double kSpeedUpPos = -0.12;
  public static final double kFullyClimbed = kArmMaxFwd;
  public static final double kClimbOpenLoopFastSpeed = 12.0;
  public static final double kClimbOpenLoopSpeed = 8.0;
  
  // Cage location info
  public static final double kCenterX = DriveConstants.kFieldMaxX / 2.0;
  public static final double kCenterY = DriveConstants.kFieldMaxY / 2.0;
  public static final double[] kCageOffsetY = {1.0541, 2.143125, 3.235325};
  public static final double kCageOffsetX = 1.2;

  public static final double kClimbRobotOffset = 0.0075;

  public static final double kFinalDriveVx = 0.25;
  public static final double kMaxDistX = 3;
  // Align end conditions
  public static final double kCloseEnoughX = 0.1;
  public static final double kCloseEnoughY = 0.03;
  public static final double kClimbAngleGood = RobotStateConstants.kClimbAngleSmall;
  public static final double kFastClimbAfter = 3;

  public static TalonFXConfiguration getPivotArmFxConfig() {
    TalonFXConfiguration armFxConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(10)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerLimit(30)
            .withSupplyCurrentLowerTime(1)
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
            .withNeutralMode(NeutralModeValue.Brake);
    armFxConfig.MotorOutput = motorOut;

    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(kCANcoderId);
    armFxConfig.Feedback = feedbackConfigs;

    return armFxConfig;
  }

  public static CurrentLimitsConfigs getZeroCurrentLimit() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    return config;
  }

  public static CurrentLimitsConfigs getRunCurrentLimit() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    return config;
  }
}
