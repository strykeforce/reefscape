package frc.robot.constants;

public class PathHandlerConstants {
  public static final String[][] kShallowPathNames = {
    {
      "ATofetch",
      "BTofetch",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "GTofetch",
      "HTofetch",
      "ITofetch",
      "JTofetch",
      "KTofetch",
      "LTofetch"
    },
    {
      "fetchToA",
      "fetchToB",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "fetchToF",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "fetchToI",
      "fetchToJ",
      "fetchToK",
      "fetchToL"
    }
  };

  public static final String[][] kProcessorShallowPathNames = {
    {
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "CTofetchP",
      "DTofetchP",
      "ETofetchP",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath"
    },
    {
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "fetchPToC",
      "fetchPToD",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath"
    }
  };

  public static final String[][] kDeepPathNames = {
    {
      "ATofetchD",
      "BTofetchD",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "KTofetchD",
      "LTofetchD"
    },
    {
      "fetchDToA",
      "fetchDToB",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "FiveMeterTestPath",
      "fetchDToK",
      "fetchDToL"
    }
  };

  public static final double kWaitingTime = 2.0;
  public static final double kServoRadius = 1.7;
  public static final double kMaxServoErrorY = 0.4;
}
