package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final Color invertRedGreen(Color color) {
    return new Color(color.green, color.red, color.blue);
  }

  public static final int kLEDPort = 0;
  // Auto/Operator = 27
  // other = 21.5
  public static final int kTotalStripLength = 51 + 10;
  public static final int kBottomStripLength = 34;
  public static final int kTopStripLength = 17;
  public static final int kFunnelStripLength = 10;
  public static final int kTopFirstIndex = kBottomStripLength;
  public static final int kFunnelFirstIndex = kTopFirstIndex + kTopStripLength;
  public static final int kAlgeaEnd = kBottomStripLength / 3 + 1;

  public static final int kLevelStart = kTopFirstIndex;
  public static final int kPlaceStart = kTopFirstIndex + kTopStripLength / 3;
  public static final int kGetAlgeaStart = kTopFirstIndex + kTopStripLength / 3 * 2;
  public static final int kAlgeaHeightStart = kFunnelFirstIndex;
  public static final int kAutoPlacingStart = kBottomStripLength / 3 * 2 + 1;

  public static final Color kAlmostBlack = new Color(0, 0, 1);

  // Normal LED Colors
  public static final Color kHasAlgea = invertRedGreen(Color.kTeal);
  public static final Color kNotHasAlgea = kAlmostBlack;

  public static final Color kCoralNotInRobot = invertRedGreen(Color.kOrange);
  public static final Color kCoralInFunnel = invertRedGreen(Color.kPurple);
  public static final Color kCoralInRobot = invertRedGreen(Color.kAntiqueWhite);

  public static final Color kL1 = invertRedGreen(Color.kRed);
  public static final Color kL2 = invertRedGreen(Color.kLightYellow);
  public static final Color kL3 = invertRedGreen(Color.kGreen);
  public static final Color kL4 = invertRedGreen(Color.kBlue);

  public static final Color kManual = kAlmostBlack;
  public static final Color kRight = invertRedGreen(Color.kOrangeRed);
  public static final Color kLeft = invertRedGreen(Color.kPurple);

  public static final Color kGetAlgea = invertRedGreen(Color.kTeal);
  public static final Color kNotGetAlgea = kAlmostBlack;

  public static final Color kCurrentLimiting = invertRedGreen(Color.kRed);

  public static final Color kAutoPlacing = invertRedGreen(Color.kBlue);

  // Climb LED Colors
  public static final Color kTooFar = invertRedGreen(Color.kRed);
  public static final Color kGood = invertRedGreen(Color.kGreen);
  public static final Color kTooClose = invertRedGreen(Color.kBlue);

  // HP Load Color
  public static final Color kLoadCoral = invertRedGreen(Color.kBlue);

  // Algea Height Colors
  public static final Color kAlgeaLow = invertRedGreen(Color.kSaddleBrown);
  public static final Color kAlgeaHigh = invertRedGreen(Color.kDarkSlateBlue);

  public static final Color[] kGameColors = {
    invertRedGreen(Color.kBlack), // a dummy color
    invertRedGreen(Color.kDarkRed),
    invertRedGreen(Color.kDarkGreen),
    invertRedGreen(Color.kDarkBlue),
    invertRedGreen(Color.kPink),
    invertRedGreen(Color.kLightGreen),
    invertRedGreen(Color.kLightBlue)
  };
}
