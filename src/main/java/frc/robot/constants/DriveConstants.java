package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
  public static final double kAlgaeRemovalSpeed = 0;

  public static final double kDeadbandAllStick = 0.075;
  public static final double kExpoScaleYawFactor = 0.75;
  public static final double kRateLimitFwdStr = 3.5;
  public static final double kRateLimitYaw = 8.0;

  public static final double kDriveMotorOutputGear = 22;
  public static final double kDriveInputGear = 52;
  public static final double kBevelInputGear = 15;
  public static final double kBevelOutputGear = 45;

  public static final double kDriveGearRatio =
      (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

  public static final double kWheelDiameterInches = 4.0;
  public static final double kMaxSpeedMetersPerSecond = 3.384;
  public static final double kSpeedStillThreshold = 0.1; // meters per second
  public static final double kGyroRateStillThreshold = 10.0; // 25  5 degrees per second
  public static final double kGyroDifferentThreshold = 5.0; // 5 degrees
  public static final int kGyroDifferentCount = 3;

  public static final double kRobotLength = 0.6223;
  public static final double kRobotWidth = 0.6223;
  public static final double kFieldMaxX = 17.526;

  public static final double kPOmega = 4.5;
  public static final double kIOmega = 0.0;
  public static final double kDOmega = 0.0;
  public static final double kMaxVelOmega =
      (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0)) / 2.0;
  public static final double kMaxAccelOmega = 5.0;

  public static final double kPHolonomic = 3.0; // was 3
  public static final double kIHolonomic = 0.0000;
  public static final double kDHolonomic = 0.00; // kPHolonomic/100

  public static Translation2d[] getWheelLocationMeters() {
    final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
    final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
    Translation2d[] locs = new Translation2d[4];
    locs[0] = new Translation2d(x, y); // left front
    locs[1] = new Translation2d(x, -y); // right front
    locs[2] = new Translation2d(-x, y); // left rear
    locs[3] = new Translation2d(-x, -y); // right rear
    return locs;
  }

  // temp stuff
  public static final int kTempAvgCount = 25;
  public static final double kTripTemp = 1300;
  public static final double kRecoverTemp = 1290;
  public static final double kNotifyTemp = 1295;

  public static final Pose2d kResetOdomPose =
      new Pose2d(new Translation2d(0.5, 3.62), Rotation2d.fromDegrees(67));

  // public static TalonFXSConfiguration
  //     getAzimuthTalonConfig() { // will be changed to a TalonFXConfiguration
  //   // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
  //   TalonFXSConfiguration azimuthConfig = new TalonFXSConfiguration();

  //   HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
  //   hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
  //   hardwareLimitSwitchConfigs.ReverseLimitEnable = false;
  //   azimuthConfig.HardwareLimitSwitch = hardwareLimitSwitchConfigs;

  //   CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
  //   currentConfig.SupplyCurrentLowerTime = 0;
  //   currentConfig.SupplyCurrentLowerLimit = 0;

  //   currentConfig.SupplyCurrentLimit = 10;
  //   currentConfig.SupplyCurrentLimitEnable = true;

  //   azimuthConfig.CurrentLimits = currentConfig;

  //   Slot0Configs slot0Config = new Slot0Configs();
  //   slot0Config.kP = 360.35;
  //   slot0Config.kI = 0.0;
  //   slot0Config.kD = 3.604;

  //   azimuthConfig.Slot0 = slot0Config;

  //   ExternalFeedbackConfigs externalFeedbackConfigs = new ExternalFeedbackConfigs();
  //   externalFeedbackConfigs.VelocityFilterTimeConstant = 0.1;
  //   externalFeedbackConfigs.ExternalFeedbackSensorSource =
  //       ExternalFeedbackSensorSourceValue.PulseWidth;
  //   azimuthConfig.ExternalFeedback = externalFeedbackConfigs;

  //   VoltageConfigs voltageConfig = new VoltageConfigs();
  //   voltageConfig.SupplyVoltageTimeConstant = 3.2; // FIXME, seems very long
  //   azimuthConfig.Voltage = voltageConfig;

  //   MotionMagicConfigs motionConfig = new MotionMagicConfigs();
  //   motionConfig.MotionMagicCruiseVelocity = 800;
  //   motionConfig.MotionMagicAcceleration = 10_000;
  //   azimuthConfig.MotionMagic = motionConfig;

  //   MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
  //   motorConfigs.DutyCycleNeutralDeadband = 0.04;
  //   motorConfigs.NeutralMode = NeutralModeValue.Coast;
  //   azimuthConfig.MotorOutput = motorConfigs;

  //   CommutationConfigs commutationConfigs = new CommutationConfigs();
  //   commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;

  //   azimuthConfig.Commutation = commutationConfigs;

  //   return azimuthConfig;
  // }

  public static TalonSRXConfiguration
      getAzimuthTalonConfig() { // will be changed to a TalonFXConfiguration
    // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

    azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

    azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;

    azimuthConfig.slot0.kP = 15.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 150.0;
    azimuthConfig.slot0.kF = 1.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.slot0.maxIntegralAccumulator = 0;

    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    azimuthConfig.voltageCompSaturation = 12;
    azimuthConfig.voltageMeasurementFilter = 32;
    azimuthConfig.neutralDeadband = 0.04;
    return azimuthConfig;
  }

  public static TalonFXConfiguration getDriveTalonConfig() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 60;

    currentConfig.StatorCurrentLimit = 140;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = true;

    driveConfig.CurrentLimits = currentConfig;

    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = 0.5; // 0.16 using phoenix 6 migrate
    slot0Config.kI = 0.5; // 0.0002 using phoenix 6 migrate
    slot0Config.kD = 0.0;
    slot0Config.kV = 0.12; // 0.047 using phoenix 6 migrate
    driveConfig.Slot0 = slot0Config;

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.DutyCycleNeutralDeadband = 0.01;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput = motorConfigs;

    return driveConfig;
  }

  public static final int kPigeonCanID = 4;

  public static Pigeon2Configuration getPigeon2Configuration() {
    Pigeon2Configuration config = new Pigeon2Configuration();

    config.MountPose.MountPoseYaw = 0.0;
    config.MountPose.MountPoseRoll = 0.0;
    config.MountPose.MountPosePitch = 0.0;

    config.GyroTrim.GyroScalarX = 0.0;
    config.GyroTrim.GyroScalarY = 0.0;
    config.GyroTrim.GyroScalarZ = -4.55;

    return config;
  }

  public static CurrentLimitsConfigs getSafeDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 30;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = false;
    return currentConfig;
  }

  public static CurrentLimitsConfigs getNormDriveLimits() {
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 30;

    currentConfig.StatorCurrentLimit = 140;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimitEnable = true;
    return currentConfig;
  }
}
