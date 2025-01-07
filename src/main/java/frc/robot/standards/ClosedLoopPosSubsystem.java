package frc.robot.standards;

import edu.wpi.first.units.measure.Angle;

public interface ClosedLoopPosSubsystem {

  public void setPosition(Angle position);

  public Angle getPosition();

  public boolean isFinished();

  public void zero();
}
