package frc.robot.constants;

public class BattMonConstants {
  // All of these aren't right and will need to be determined

  // Battery Voltage Conversion
  public static final double kBattVolt1 = 12;
  public static final double kBattVolt1DC = 0.9;
  public static final double kBattVolt2 = 2;
  public static final double kBattVolt2DC = 0.1;
  public static final double kBattVoltSlope =
      (kBattVolt1 - kBattVolt2) / (kBattVolt1DC - kBattVolt2DC);
  public static final double kBattVoltOffset = kBattVolt1 - kBattVoltSlope * kBattVolt1DC;

  // Battery Current Conversion
  public static final double kBattCurrent1 = 50;
  public static final double kBattCurrent1DC = 0.9;
  public static final double kBattCurrent2 = 1;
  public static final double kBattCurrent2DC = 0.1;
  public static final double kBattCurrentSlope =
      (kBattCurrent1 - kBattCurrent2) / (kBattCurrent1DC - kBattCurrent2DC);
  public static final double kBattCurrentOffset =
      kBattCurrent1 - kBattCurrentSlope * kBattCurrent1DC;

  // PDP Voltage Conversion
  public static final double kPdpVoltage1 = 42;
  public static final double kPdpVoltage1DC = 0.9;
  public static final double kPdpVoltage2 = 2;
  public static final double kPdpVoltage2DC = 0.1;
  public static final double kPdpVoltSlope =
      (kPdpVoltage1 - kPdpVoltage2) / (kPdpVoltage1DC - kPdpVoltage2DC);
  public static final double kPdpVoltOffset = kPdpVoltage1 - kPdpVoltSlope * kPdpVoltage1DC;

  // Breaker Temp Conversion
  public static double kBreakerTemp1 = 0.32;
  public static double kBreakerTemp2 = 0.0047;

  // Low Battery Thresholds
  public static final double kBatt1Low = 2;
  public static final double kBatt2Low = 2;

  // Temp Limiting
  public static final double kCurrent1 = 10;
  public static final double kCurrent1Temp = 200;
  public static final double kCurrent2 = 200;
  public static final double kCurrent2Temp = 160;
  public static final double kTempLimitSlope =
      (kCurrent1Temp - kCurrent2Temp) / (kCurrent1 - kCurrent2);
  public static final double kTempLimitOffset = kCurrent1Temp - kTempLimitSlope * kCurrent1;
  public static final double kHysteresis = 20;
  public static final double kWarningOffset = 5;

  public static final double kPDPHigh = 42;
  public static final double kTempHighSlope = 10;
  public static final double kTempDangerSlope = 15;
}
