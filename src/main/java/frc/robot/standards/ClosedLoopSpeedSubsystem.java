package frc.robot.standards;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ClosedLoopSpeedSubsystem {

  public void setSpeed(AngularVelocity speed);

  public AngularVelocity getSpeed();

  public boolean atSpeed();
}
