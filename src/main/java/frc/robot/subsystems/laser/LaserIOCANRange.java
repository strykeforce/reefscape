package frc.robot.subsystems.laser;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.constants.LaserConstants;

public class LaserIOCANRange implements LaserIO {
  private CANrange laser;

  public LaserIOCANRange() {
    laser = new CANrange(LaserConstants.LaserCanId);
    laser.getConfigurator().apply(LaserConstants.getLaserConfig());
  }

  @Override
  public void updateInputs(LaserIOInputs inputs) {
    inputs.distance = getDistanceMeters();
  }

  public double getDistanceMeters() {
    if (laser.getIsDetected().getValue()) {
      return laser.getDistance().getValue().in(Meters);
    } else {
      return -1.0;
    }
  }
}
