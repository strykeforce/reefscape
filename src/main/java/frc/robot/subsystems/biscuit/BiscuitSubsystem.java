// Blessed by the great tech-priests of the Adeptus Mechanicus

package frc.robot.subsystems.biscuit;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.RobotConstants;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class BiscuitSubsystem extends MeasurableSubsystem {

  private org.slf4j.Logger logger;
  private BiscuitIO io;
  private BiscuitIOInputsAutoLogged inputs = new BiscuitIOInputsAutoLogged();
  private Angle setPoint = Rotations.of(0);
  private boolean hasZeroed = false;

  public BiscuitSubsystem(BiscuitIO io) {
    this.logger = LoggerFactory.getLogger(this.getClass());
    this.io = io;
    hasZeroed = io.zero();
  }

  public boolean hasZeroed() {
    return hasZeroed;
  }

  public void setIsRemovingAlgae(boolean isRemoving) {
    io.setIsRemovingAlgae(isRemoving);
  }

  public void setPosition(Angle position, boolean hasAlgae) {
    io.setPosition(position, hasAlgae);
    setPoint = position;
  }

  public Angle getPosition() {
    return Rotations.of(inputs.position);
  }

  public AngularVelocity getVelocity(AngularVelocity velocity) {
    return RotationsPerSecond.of(inputs.velocity);
  }

  public void zero() {
    hasZeroed = io.zero();
  }

  public boolean isFinished() {
    return Math.abs(getPosition().minus(setPoint).in(Rotations)) < BiscuitConstants.kCloseEnough;
  }

  public boolean isSafeToStow() {
    return getPosition().in(Rotations) < RobotConstants.kSafeToStowUpper
        && getPosition().in(Rotations) > RobotConstants.kSafeToStowLower;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(getName(), inputs);
    Logger.recordOutput("Biscuit setPoint", setPoint.in(Rotations));
    Logger.recordOutput("Is Biscuit Finished", isFinished() ? 1.0 : 0.0);

    // double pos = MathUtil.inputModulus(inputs.rawPulseWidth, 0, 1);
    // double error =
    //     Math.abs(BiscuitConstants.kTicksPerRot * (RobotConstants.kZero - pos) - inputs.position);

    // if (setPoint == BiscuitConstants.kStowSetpoint
    //     && isFinished()
    //     && inputs.velocity < BiscuitConstants.kRezeroVelocityCloseEnough
    //     && error >= BiscuitConstants.kRezeroErrorCloseEnough) {
    //   logger.info("Rezeroing Biscuit");
    //   zero();
    // }
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
