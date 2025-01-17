package frc.robot.subsystems.battMon;

import org.littletonrobotics.junction.AutoLog;

public interface BattMonIO {

  @AutoLog
  public class BattMonIOInputs {
    public double batteryVoltage;
    public double batteryCurrent;
    public double pdpVoltage;
    public double breakerTemp;
  }

  public default void updateInputs(BattMonIOInputs inputs) {}
}
