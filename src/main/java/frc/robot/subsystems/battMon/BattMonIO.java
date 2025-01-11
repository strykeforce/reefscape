package frc.robot.subsystems.battMon;

import org.littletonrobotics.junction.AutoLog;

public interface BattMonIO {

  @AutoLog
  public class BattMonIOInputs {
    public double batt1Output = 0;
    public double batt2Output = 0;
    public double pdpOutput = 0;
    public double tempOutput = 0;
  }

  public default void updateInputs(BattMonIOInputs inputs) {}
}
