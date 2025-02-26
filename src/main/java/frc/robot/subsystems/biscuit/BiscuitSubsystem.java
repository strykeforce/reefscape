// Blessed by the great tech-priests of the Adeptus Mechanicus

package frc.robot.subsystems.biscuit;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.BiscuitConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class BiscuitSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {

  private BiscuitIO io;
  private BiscuitIOInputsAutoLogged inputs = new BiscuitIOInputsAutoLogged();
  private Angle setPoint = Rotations.of(0);

  public BiscuitSubsystem(BiscuitIO io) {
    this.io = io;
  }

  @Override
  public void setPosition(Angle position) {
    io.setPosition(position);
    setPoint = position;
  }

  @Override
  public Angle getPosition() {
    return Rotations.of(inputs.position);
  }

  public AngularVelocity getVelocity(AngularVelocity velocity) {
    return RotationsPerSecond.of(inputs.velocity);
  }

  @Override
  public void zero() {
    io.zero();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getPosition().minus(setPoint).in(Rotations)) < BiscuitConstants.kCloseEnough;
  }

  public boolean isSafeToStow() {
    return getPosition().in(Rotations) < BiscuitConstants.kSafeToStowUpper
        && getPosition().in(Rotations) > BiscuitConstants.kSafeToStowLower;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput("Biscuit setPoint", setPoint.in(Rotations));
    Logger.recordOutput("Is Biscuit Finished", isFinished() ? 1.0 : 0.0);

    if(inputs.position == BiscuitConstants.kCloseEnough && offset > 0) {
      zero();
      Logger.error("error");
    }
  }

  @Override
  public void registerWith(TelemetryService telemetry) {
    super.registerWith(telemetry);
    io.registerWith(telemetry);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Is Biscuit Finished", () -> isFinished() ? 1.0 : 0.0),
        new Measure("Biscuit Set Point", () -> setPoint.in(Rotations)));
  }
}
