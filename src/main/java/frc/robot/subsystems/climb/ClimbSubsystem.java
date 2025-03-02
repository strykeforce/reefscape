// still have a non-negligible amount of stuff to do on the state machine

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ClimbConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbState;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final ClimbIO io;

  private ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

  private boolean isRotatingToPosition = false;
  private boolean isClimbZeroed = false;
  private boolean isRatchetOn = false;
  private boolean inPosition = false;
  private boolean isBeamBroken = false;
  private boolean isPinDeployed = false;
  private boolean proceedToClimb = false;
  private int climbZeroStableCounts; // idk what this means
  // more

  private Angle setpoints = Rotations.of(0.0);
  private ClimbState curState = ClimbState.INIT;
  private Timer hangTimer = new Timer();

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;
    enableRatchet(false);
    zero();
  }

  @Override
  public Angle getPosition() {
    return climbInputs.position;
  }

  @Override
  public void setPosition(Angle position) {
    io.setPosition(position);
    setpoints = position;
  }

  @Override
  public boolean isFinished() {
    return setpoints.minus(climbInputs.position).abs(Rotations)
        <= ClimbConstants.kPivotArmCloseEnough.in(Rotations);
  }

  @Override
  public void zero() {
    io.zero();
  }

  private void setClimbDebugMsg(String msg) {
    Logger.recordOutput("Climb/DebugMsg", msg);
  }

  public void toggleClimbRatchet() {
    enableRatchet(!isRatchetOn);
  }

  public void enableRatchet(boolean enable) {
    if (enable) {
      io.setRatchetServoPosition(ClimbConstants.kRatchetEngatedPos); // fixme
      setClimbDebugMsg("Engaging Ratchet");
      isRatchetOn = true;
    } else {
      io.setRatchetServoPosition(ClimbConstants.kRatchetDisengagedPos);
      setClimbDebugMsg("Disengaging Ratchet");
      isRatchetOn = false;
    }
  }

  public Angle getSetpoints() {
    return setpoints;
  }

  public ClimbState getState() {
    return curState;
  }

  public void prepClimb() {}

  public void climb() {}

  public void deployPin(boolean enable) {
    if (enable) {
      io.setPinServoPosition(ClimbConstants.kPinDeployedPosition);
      setClimbDebugMsg("Deploying Pin");
      isPinDeployed = true;
    } else {
      io.setPinServoPosition(ClimbConstants.kPinRetractedPosition);
      setClimbDebugMsg("Retracting Pin");
      isPinDeployed = true;
    }
  }

  public void toggleDeployState() {
    deployPin(!isPinDeployed);
  }

  public boolean isClimbFinished() {
    return curState == ClimbState.CLIMBED;
  }

  public void descend() {}

  public void stow() {}

  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(climbInputs);
    Logger.processInputs(getName(), climbInputs);
    // climbInputs.ratchetPosition = ratchetServo.getPosition();

    // State Machine
    switch (curState) {
      case INIT:
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Climb/curState", curState.ordinal()); // works not
    Logger.recordOutput("Climb/setpoint", setpoints.in(Rotations));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public enum ClimbState {
    INIT,
    STOWED,
    PREP_CLIMBING,
    PREPPED,
    CLIMBING,
    CLIMBED
  }
}
