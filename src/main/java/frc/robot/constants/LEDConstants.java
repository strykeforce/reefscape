package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  // Auto/Operator = 27
  // other = 21.5
  public static final int kTotalStripLength = 70;
  public static final int kBottomStripLength = 43;
  public static final int kTopStripLength = 27;
  public static final int kTopFirstIndex = kBottomStripLength + 1;

  public static final int kAlgeaEnd = kBottomStripLength / 3 + 1;
  public static final int kLevelStart = kTopFirstIndex;
  public static final int kPlaceStart = kTopFirstIndex + kTopStripLength / 3;
  public static final int kGetAlgeaStart = kTopFirstIndex + kTopStripLength / 3 * 2;
  public static final int kAutoPlacingStart = kTopFirstIndex;

  public static final Color kAlmostBlack = new Color(0, 0, 1);

  // Normal LED Colors
  public static final Color kHasAlgea = Color.kAquamarine;
  public static final Color kNotHasAlgea = kAlmostBlack;

  public static final Color kCoralNotInRobot = Color.kOrange;
  public static final Color kCoralInFunnel = Color.kHotPink;
  public static final Color kCoralInRobot = Color.kWhite;

  public static final Color kL1 = Color.kBlue;
  public static final Color kL2 = Color.kGreen;
  public static final Color kL3 = Color.kYellow;
  public static final Color kL4 = Color.kRed;

  public static final Color kManual = kAlmostBlack;
  public static final Color kRight = Color.kSaddleBrown;
  public static final Color kLeft = Color.kGreenYellow;

  public static final Color kGetAlgea = Color.kAquamarine;
  public static final Color kNotGetAlgea = kAlmostBlack;

  public static final Color kCurrentLimiting = Color.kRed;

  public static final Color kAutoPlacing = Color.kPurple;

  // Climb LED Colors
  public static final Color kWaitingForCage = Color.kRed;
  public static final Color kHasCage = Color.kGreen;
  public static final Color kClimbed = Color.kGoldenrod;
  public static final Color[] kGameColors = {
    Color.kBlack, // a dummy color
    Color.kDarkRed,
    Color.kDarkGreen,
    Color.kDarkBlue,
    Color.kPink,
    Color.kLightGreen,
    Color.kLightBlue
  };
}
