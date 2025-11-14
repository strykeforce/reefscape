package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FunnelConstants;
import frc.robot.standards.OpenLoopSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class FunnelSubsystem extends MeasurableSubsystem implements OpenLoopSubsystem {

  // Private Variables
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
  private float totalBreaks = 0;
  private Timer loopTimer = new Timer();
  private long startTime = 0;

  public FunnelState curState = FunnelState.HasNotSeenCoral;

  // Constructor
  public FunnelSubsystem(FunnelIO io) {
    loopTimer.start();
    this.io = io;
  }

  // Getter/Setter Methods
  public FunnelState getState() {
    return curState;
  }

  public boolean hasCoral() {
    // return true;
    return curState == FunnelState.HasSeenCoral;
  }

  @Override
  public void periodic() {
    loopTimer.reset();
    loopTimer.start();
    startTime = RobotController.getFPGATime();
    // Update Inputs
    io.updateInputs(inputs);
    Logger.processInputs("funnelInputs", inputs);

    switch (curState) {
      case HasSeenCoral:
        break;
      case HasNotSeenCoral:
        if (inputs.isRevBeamBroken == true) {
          totalBreaks += 1;
        } else {
          totalBreaks = 0;
        }

        if (totalBreaks >= FunnelConstants.kFunnelBeamCounts) {
          curState = FunnelState.HasSeenCoral;
        }
        break;
    }

    // Log Outputs
    Logger.recordOutput("Funnel/curState", curState);
    Logger.recordOutput("Funnel/totalBreaks", totalBreaks);
    Logger.recordOutput("Funnel/LoopTime", (RobotController.getFPGATime() - startTime));
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  // State Enum
  public enum FunnelState {
    HasSeenCoral,
    HasNotSeenCoral
  }

  @Override
  public void setPercent(double pct) {
    io.setPct(pct);
  }

  public void startMotor() {
    setPercent(FunnelConstants.kFunnelPercentOutput);
  }

  public void stopMotor() {
    setPercent(0.0);
  }

  public void reverse() {
    setPercent(-0.5);
  }

  public void clearCoral() {
    curState = FunnelState.HasNotSeenCoral;
  }
}
