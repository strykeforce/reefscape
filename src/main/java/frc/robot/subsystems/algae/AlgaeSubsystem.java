package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.ExampleConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;
import frc.robot.subsystems.algae.algaeIO.IOInputs;

import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class AlgaeSubsystem implements ClosedLoopPosSubsystem {
    private final algaeIO io;
  private final IOInputsAutoLogged inputs = new IOInputsAutoLogged();
  private Angle setpoint = Rotations.of(0.0);

  public AlgaeSubsystem(algaeIO io) {
    this.io = io;
  }

  public Angle getPosition() {
    return getPosition();
  }

  public void setPosition(Angle position) {
    setPosition(position);
    setpoint = position;
  }

  public void zero(){
    setPosition(Rotations.of(0));
  }

  public boolean isFinished() {
    return setpoint.minus(getPosition()).abs(Rotations) <= AlgaeConstants.kCloseEnough.in(Rotations);
  }

  // Periodic Function
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    Logger.recordOutput("Algae/setpoint", setpoint.in(Rotations));
  }

  // Grapher
  public void registerWith(TelemetryService telemetryService) {

    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }
}