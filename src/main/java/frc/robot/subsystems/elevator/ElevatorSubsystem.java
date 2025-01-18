package frc.robot.subsystems.elevator;

import frc.robot.constants.ElevatorConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;

import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.units.measure.Angle;


public class ElevatorSubsystem implements ClosedLoopPosSubsystem extends MeasurableSubsystem {
    // Private Variables
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ExiterIOInputsAutoLogged();

    private double setpoints;

    // Constructor
    public void ExiterSubsystem(ElevatorIO io) {
        this.io = io;
  }

  @Override
  public void periodic() {
    //Read inputs
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("ElevatorInputs", inputs);

    //Log outputs
    Logger.recordOutput("Elevator/setpoints", setpoints);
    Logger.recordOutput("Exiter/atSpeed", atSpeed());
  }

   // Grapher
   @Override
   public void registerWith(TelemetryService telemetryService) {
 
     super.registerWith(telemetryService);
     io.registerWith(telemetryService);
   }

  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public double getSpeeds() {
    return setpoints;
  }
  
  public boolean atSpeed() {
    return Math.abs(setpoints - inputs.velocityLeft) <= ElevatorConstants.kCloseEnough);
  }

  public void setPosition(Angle position) {}

  public Angle getPosition() {}

  public boolean isFinished() {}

  public default void zero() {
    
  }

}
