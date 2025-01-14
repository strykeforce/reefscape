package frc.robot.subsystems.battMon;

import org.littletonrobotics.junction.AutoLog;

public interface BattMonIO {

  @AutoLog
  public class BattMonIOInputs {
    public double batt1Output;
    public double batt2Output;
    public double pdpOutput;
    public double tempOutput;
  }

  public default void updateInputs(BattMonIOInputs inputs) {}
}
