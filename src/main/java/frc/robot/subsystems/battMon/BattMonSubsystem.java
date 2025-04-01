package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BattMonSubsystem extends SubsystemBase {
  // Private objects
  private BattMonIO io;
  private BattMonIOInputsAutoLogged inputs = new BattMonIOInputsAutoLogged();
  private battMonState curState;
  // Alerts
  private Alert highTempAlert = new Alert("WARNING reaching maximum temp!", AlertType.kWarning);
  private Alert dangerTempAlert = new Alert("DANGER MAXIMUM TEMP REACHED", AlertType.kError);
  private Alert safeAlert = new Alert("SAFE Temp low. Limits removed", AlertType.kInfo);

  private double recoveryTempThreshold;
  private double curDangerThreshold;
  private double curWarnThreshold;

  @Override
  public void periodic() {
    // Refresh data and graph it
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    // curDangerThreshold =
    //     BattMonConstants.kTempLimitSlope * inputs.batteryCurrent
    //         + BattMonConstants.kTempLimitOffset;
    // curWarnThreshold = curDangerThreshold - BattMonConstants.kWarningOffset;

    // // Check for dangerous outputs NOTE all of these values are temporary
    // switch (curState) {
    //   case NORMAL:
    //     if (inputs.breakerTemp >= curWarnThreshold) {
    //       highTempAlert.set(true);
    //       safeAlert.set(false);
    //       recoveryTempThreshold = curWarnThreshold - BattMonConstants.kHysteresis;
    //       curState = battMonState.WARNING;
    //     }
    //     break;

    //   case WARNING:
    //     if (inputs.breakerTemp > curDangerThreshold) {
    //       highTempAlert.set(false);
    //       dangerTempAlert.set(true);
    //       curState = battMonState.DANGER;
    //     } else if (inputs.breakerTemp < recoveryTempThreshold) {
    //       highTempAlert.set(false);
    //       safeAlert.set(true);
    //       curState = battMonState.NORMAL;
    //     }
    //     break;

    //   case DANGER:
    //     if (inputs.breakerTemp <= recoveryTempThreshold) {
    //       dangerTempAlert.set(false);
    //       highTempAlert.set(true);
    //       curState = battMonState.WARNING;
    //     }
    // }
  }

  public battMonState getState() {
    return curState;
  }

  public enum battMonState {
    NORMAL, // Safe Temp
    WARNING, // Reaching Dangerous temps
    DANGER // Dangerous temps
  }
}
