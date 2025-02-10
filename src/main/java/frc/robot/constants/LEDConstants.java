package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final int kStripLength = 42;

  public static final int kAlgeaEnd = kStripLength / 3;
  public static final int kLevelStart = kStripLength / 3;
  public static final int kPlaceStart = kStripLength / 9 * 7;
  public static final int kGetAlgeaStart = kStripLength / 9 * 8;

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
