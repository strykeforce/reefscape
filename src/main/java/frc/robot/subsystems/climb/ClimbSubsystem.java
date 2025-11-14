// still have a non-negligible amount of stuff to do on the state machine

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
  private boolean spedUp = false;
  // more

  private double setpoints = 0.0;
  private ClimbState curState = ClimbState.INIT;
  private Timer hangTimer = new Timer();
  private Timer loopTimer = new Timer();
  private long startTime = 0;

  public ClimbSubsystem(ClimbIO io) {
    loopTimer.start();
    this.io = io;
    Logger.recordOutput("Climb/isDesparate", false);
    enableRatchet(false);
    deployClimb(false);
    zero();
  }

  @Override
  public Angle getPosition() {
    return Rotations.of(climbInputs.position);
  }

  @Override
  public void setPosition(Angle position) {
    io.setPosition(position);
    setpoints = position.in(Rotations);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(setpoints - climbInputs.position) < ClimbConstants.kPivotArmCloseEnough);
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
      io.setRatchetServoPosition(ClimbConstants.kRatchetEngagedPos); // fixme
      setClimbDebugMsg("Engaging Ratchet");
      isRatchetOn = true;
    } else {
      io.setRatchetServoPosition(ClimbConstants.kRatchetDisengagedPos);
      setClimbDebugMsg("Disengaging Ratchet");
      isRatchetOn = false;
    }
  }

  public double getSetpoints() {
    return setpoints;
  }

  public ClimbState getState() {
    return curState;
  }

  public void prepClimb() {
    enableRatchet(false);
    deployClimb(true);
    setState(ClimbState.PREPPED);
  }

  public void climb() {
    if (curState == ClimbState.PREPPED) {
      // setPosition(ClimbConstants.kClimbCagePos);
      if (DriverStation.getMatchTime() <= ClimbConstants.kFastClimbAt) {
        Logger.recordOutput("Climb/isDesparate", true);
        io.setPercent(ClimbConstants.kClimbOpenLoopFastSpeed);
        spedUp = true;
      } else {
        Logger.recordOutput("Climb/isDesparate", false);
        io.setPercent(ClimbConstants.kClimbOpenLoopSpeed);
        spedUp = false;
      }
      setState(ClimbState.CLIMBING);
    }
  }

  private void deployClimb(boolean enable) {
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
    deployClimb(!isPinDeployed);
  }

  public boolean isClimbFinished() {
    return curState == ClimbState.CLIMBED;
  }

  public void stow() {}

  private void setState(ClimbState state) {
    setClimbDebugMsg(curState + "->" + state);
    setpoints = ClimbConstants.kFullyClimbed;
    curState = state;
  }

  @Override
  public void periodic() {
    loopTimer.reset();
    loopTimer.start();
    startTime = RobotController.getFPGATime();
    // Read Inputs
    io.updateInputs(climbInputs);
    Logger.processInputs(getName(), climbInputs);
    // climbInputs.ratchetPosition = ratchetServo.getPosition();

    // State Machine
    switch (curState) {
      case INIT:
        break;
      case PREPPED:
        break;
      case CLIMBING:
        if (climbInputs.position >= ClimbConstants.kClimbRatchedEngage && !isRatchetOn) {
          enableRatchet(true);
        }
        // if (climbInputs.position >= ClimbConstants.kSpeedUpPos && !spedUp) {
        //   io.setPercent(ClimbConstants.kClimbOpenLoopFastSpeed);
        //   spedUp = true;
        // }
        if (isFinished()) {
          io.setCoastMode(true);
          setState(ClimbState.CLIMBED);
          io.setPercent(0.0);
        }
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Climb/curState", curState);
    Logger.recordOutput("Climb/setpoint", setpoints);
    Logger.recordOutput("Climb/loopTime", (RobotController.getFPGATime() - startTime));
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
