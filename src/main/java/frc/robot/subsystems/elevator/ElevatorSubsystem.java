package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ElevatorConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElevatorSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  // Private Variables
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorStates currState = ElevatorStates.ZEROED;

  private Angle setpoints;

  private int zeroCounter = 0;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void setPosition(Angle position) {
    setpoints = position;
    io.setPosition(position);
  }

  @Override
  public Angle getPosition() {
    return setpoints;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getPosition().minus(setpoints).in(Rotations))
            < ElevatorConstants.kCloseEnoughRotations
        && currState != ElevatorStates.ZEROING;
  }

  @Override
  public void zero() {
    // FIXME: immediately zeroed for testing
    io.zero();

    // currState = ElevatorStates.ZEROING;
    // io.setCurrentLimitConfig(ElevatorConstants.getZeroingCurrentLimitsConfigs());
    // io.setSoftLimitConfig(ElevatorConstants.getZeroingSoftLimitConfigs());
    // io.setVelocityOpenLoop(ElevatorConstants.kZeroSpeed);
  }

  @Override
  public void periodic() {
    // Read inputs
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("ElevatorInputs", inputs);

    // Log outputs
    Logger.recordOutput("Elevator/setpoints", setpoints);

    switch (currState) {
      case ZEROING -> {
        if (Math.abs(inputs.velocity) < ElevatorConstants.kZeroedThreshhold) {
          zeroCounter++;
          if (zeroCounter >= ElevatorConstants.kZeroCounter) {
            io.zero();
            zeroCounter = 0;
            io.setCurrentLimitConfig(ElevatorConstants.getBothFXConfig().CurrentLimits);
            io.setSoftLimitConfig(ElevatorConstants.getBothFXConfig().SoftwareLimitSwitch);
            currState = ElevatorStates.ZEROED;
          }
        } else {
          zeroCounter = 0;
        }
      }
      case ZEROED -> {}
    }
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("state", () -> currState.ordinal()));
  }

  public enum ElevatorStates {
    ZEROED,
    ZEROING
  };
}
