//still have a non-negligible amount of stuff to do on the state machine


package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ClimbConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbState;

import java.util.Set;
//import org.littletonrobotics.junction.Logger;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  private final ClimbIO climbIo;
  private final ClimbIO pivotArmIo; //will probably not need all these individual ios
  private final ClimbIO ratchetIo;
  private final ClimbIO pinIo;

  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();
  private final ClimbIOInputsAutoLogged pivotArmInputs = new ClimbIOInputsAutoLogged(); 
  private final ClimbIOInputsAutoLogged ratchetInputs = new ClimbIOInputsAutoLogged();
  private final ClimbIOInputsAutoLogged pinInputs = new ClimbIOInputsAutoLogged();

  private Logger logger = LoggerFactory.getLogger(this.getClass());

  private boolean isRotatingToPosition = false;
  private boolean isClimbZeroed = false;
  private boolean isRatchetOn = false;
  private boolean inPosition = false;
  private boolean isBeamBroken = false;
  private boolean isPinDeployed = false;
  private boolean proceedToClimb = false;
  private int climbZeroStableCounts; //idk what this means
  //more

  private Angle setpoints = Rotations.of(0.0);
  private ClimbState curState = ClimbState.INIT;
  private Timer hangTimer = new Timer();

  public ClimbSubsystem(ClimbIO climbIo, ClimbIO pivotArmIo, ClimbIO ratchetIo, ClimbIO pinIo) {
    this.climbIo = climbIo;
    this.pivotArmIo = pivotArmIo;
    this.ratchetIo = ratchetIo;
    this.pinIo = pinIo;

    enableRatchet(false);

  }

  @Override
  public Angle getPosition() {
    return climbInputs.position;
  }

  @Override
  public void setPosition(Angle position) {
    climbIo.setPosition(position);
    setpoints = position;
    
  }

  @Override
  public boolean isFinished() {
    return setpoints.minus(climbInputs.position).abs(Rotations)
        <= ClimbConstants.kPivotArmCloseEnough.in(Rotations);
  }

  @Override
  public void zero() {
    climbIo.zero();

    curState = ClimbState.ZEROED;
  }

  public void toggleClimbRatchet() {
    if (isRatchetOn) enableRatchet(false);
    else enableRatchet(true);
  }

  public void enableRatchet(boolean enable) {
    if (enable) {
      ratchetIo.setPosition(); //fixme
      logger.info("Engaging Ratchet");
      isRatchetOn = true;
    }
    else {
      ratchetIo.setPosition(); //fixme
      logger.info("Disengaging Ratchet");
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
      pinIo.setPosition(); //fixme
      logger.info("Deploying Pin");
      isPinDeployed = true;
    }
    else {
      pinIo.setPosition(); //fixme
      logger.info("Retracting Pin");
      isPinDeployed = true;
    }
    
  }

  public void togglePin() {
    if (isPinDeployed) deployPin(false);
    else enableRatchet(true);
  }

  public boolean isClimbFinished() {}

  public void descend() {}

  public void stow() {

  }

  @Override
  public void periodic() {
    // Read Inputs
    climbIo.updateInputs(climbInputs);
    pivotArmIo.updateInputs(pivotArmInputs);
    pinIo.updateInputs(pinInputs);
    ratchetIo.updateInputs(ratchetInputs);
    Logger.processInputs(getName(), climbInputs);
    //climbInputs.ratchetPosition = ratchetServo.getPosition();

    // State Machine
    switch (curState) {
      case INIT:
        break;
      case ZEROING:
        if () {

        }
        else {

        }
        break;
      default:
        break;
      case ZEROED:
      if (proceedToClimb) {
        
      }
      else
      

    }

    // Log Outputs
    Logger.recordOutput("Coral/curState", curState.ordinal()); //works not
    Logger.recordOutput("Coral/setpoint", setpoints.in(Rotations));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    pivotArmIo.registerWith(telemetryService);
    pinIo.registerWith(telemetryService);
    ratchetIo.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public enum ClimbState {
    INIT,
    STOWED,
    ZEROING,
    ZEROED,
    PREP_CLIMBING,
    PREPPED,
    CLIMBING,
    CLIMBED,
    DESCENDING,
    DOWN
  }
}
