// Blessed by the great tech-priests of the Adeptus Mechanicus

package frc.robot.subsystems.biscuit;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.BiscuitConstants;
import frc.robot.constants.RobotConstants;
import java.util.Set;
import net.jafama.FastMath;
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
  private Angle prevSetPoint = Rotations.of(0);
  private boolean hasZeroed = false;
  private int zeroCounter = 0;

  public BiscuitState curState = BiscuitState.NORMAL;

  public BiscuitSubsystem(BiscuitIO io) {
    this.logger = LoggerFactory.getLogger(this.getClass());
    this.io = io;
    hasZeroed = io.zero();
  }

  public BiscuitState getState() {
    return curState;
  }

  public boolean hasZeroed() {
    return hasZeroed;
  }

  public void setIsRemovingAlgae(boolean isRemoving) {
    io.setIsRemovingAlgae(isRemoving);
  }

  public void setPosition(Angle position, boolean hasAlgae) {
    if (position.equals(setPoint)) {
      return;
    }
    io.setPosition(position, hasAlgae);
    setPoint = position;
  }

  public Angle getPosition() {
    return Rotations.of(inputs.position);
  }

  public AngularVelocity getVelocity(AngularVelocity velocity) {
    return RotationsPerSecond.of(inputs.velocity);
  }

  public void zeroCheck() {
    zeroCounter++;
    if (!getIsEncoderBroken() && curState != BiscuitState.BROKEN) {
      hasZeroed = io.zero();
      curState = BiscuitState.NORMAL;
    } else {
      curState = BiscuitState.BROKEN;
    }
  }

  public void zero() {
    io.zero();
  }

  public boolean isFinished() {
    return Math.abs(getPosition().minus(setPoint).in(Rotations)) < BiscuitConstants.kCloseEnough;
  }

  public boolean isSafeToStow() {
    return getPosition().in(Rotations) < RobotConstants.kSafeToStowUpper
        && getPosition().in(Rotations) > RobotConstants.kSafeToStowLower;
  }

  private boolean getIsEncoderBroken() {
    double curPos = MathUtil.inputModulus(inputs.rawPulseWidth, 0, 1);
    return curPos >= BiscuitConstants.kInFunnel || curPos <= BiscuitConstants.kTooLow;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (curState) {
      case NORMAL:
        if (isFinished()
            && FastMath.abs(inputs.velocity) <= BiscuitConstants.kZeroVelThresh
            && prevSetPoint != setPoint) {
          if (FastMath.abs(RobotConstants.kMicAlgaeSetpoint.minus(setPoint).in(Rotations))
                  < BiscuitConstants.kCloseEnough
              || FastMath.abs(RobotConstants.kProcessorSetpoint.minus(setPoint).in(Rotations))
                  < BiscuitConstants.kCloseEnough) {
            curState = BiscuitState.CHECK_ZERO;
          } else {
            prevSetPoint = setPoint;
          }
        }
        break;
      case CHECK_ZERO:
        prevSetPoint = setPoint;
        zeroCheck();
        break;
      case BROKEN:
        break;
    }

    Logger.processInputs(getName(), inputs);
    Logger.recordOutput("Biscuit/setPoint", setPoint.in(Rotations));
    Logger.recordOutput("Biscuit/IsFinished", isFinished() ? 1.0 : 0.0);
    Logger.recordOutput("Biscuit/curState", curState);
    Logger.recordOutput("Biscuit/zeroCounts", zeroCounter);

    // if (setPoint != prevSetPoint && isFinished()) {
    //   zeroCheck();
    //   prevSetPoint = setPoint;
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

  public enum BiscuitState {
    NORMAL,
    CHECK_ZERO,
    BROKEN
  }
}
