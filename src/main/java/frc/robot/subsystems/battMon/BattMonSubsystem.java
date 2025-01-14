package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        if (inputs.tempOutput >= (1000 * inputs.pdpOutput)) {
          highTempAlert.set(true);
          safeAlert.set(false);
          tempThreshold = inputs.tempOutput - (100 * inputs.pdpOutput);
          curState = battMonState.WARNING;
        } else {
          break;
        }

      case WARNING:
        if (inputs.tempOutput > (2000 * inputs.pdpOutput)) {
          highTempAlert.set(false);
          dangerTempAlert.set(true);
          curState = battMonState.DANGER;
        } else if (inputs.tempOutput < tempThreshold) {
          highTempAlert.set(false);
          safeAlert.set(true);
          curState = battMonState.NORMAL;
        } else {
          break;
        }

      case DANGER:
        if (inputs.tempOutput >= (2000 * inputs.pdpOutput)) {
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
