package frc.robot.subsystems.laser;

import org.littletonrobotics.junction.AutoLog;

public interface LaserIO {

  @AutoLog
  public static class LaserIOInputs {
    public double distance = 0.0;
  }

  public void updateInputs(LaserIOInputs inputs);

  public double getDistanceMeters();
}
