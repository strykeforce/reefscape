package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BattMonConstants;
import frc.robot.subsystems.battMon.BattMonIO.BattMonIOInputs;

public class BattMonSubsystem extends SubsystemBase {
  // Private objects
  private BattMonIO io;
  private BattMonIOInputs inputs = new BattMonIOInputs();
  private battMonState curState;
  // Alerts
  private Alert highTempAlert = new Alert("WARNING reaching maximum temp!", AlertType.kWarning);
  private Alert dangerTempAlert = new Alert("DANGER MAXIMUM TEMP REACHED", AlertType.kError);
  private Alert safeAlert = new Alert("SAFE Temp low. Limits removed", AlertType.kInfo);

  private double tempThreshold;

  @Override
  public void periodic() {
    // Refresh data and graph it
    io.updateInputs(inputs);

    // Check for dangerous outputs NOTE all of these values are temporary
    switch (curState) {
      case NORMAL:
        if (inputs.breakerTemp >= (BattMonConstants.kTempHighSlope * inputs.pdpVoltage + 100)) {
          highTempAlert.set(true);
          safeAlert.set(false);
          tempThreshold = inputs.breakerTemp - BattMonConstants.kHysteresis * inputs.pdpVoltage;
          curState = battMonState.WARNING;
        } else {
          break;
        }

      case WARNING:
        if (inputs.breakerTemp > (BattMonConstants.kTempDangerSlope * inputs.pdpVoltage)) {
          highTempAlert.set(false);
          dangerTempAlert.set(true);
          curState = battMonState.DANGER;
        } else if (inputs.breakerTemp < tempThreshold) {
          highTempAlert.set(false);
          safeAlert.set(true);
          curState = battMonState.NORMAL;
        } else {
          break;
        }

      case DANGER:
        if (inputs.breakerTemp >= (BattMonConstants.kTempDangerSlope * inputs.pdpVoltage)) {
          break;
        } else {
          dangerTempAlert.set(false);
          highTempAlert.set(true);
          curState = battMonState.WARNING;
        }
    }
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
