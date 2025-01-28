package frc.robot.subsystems.elevator;

import frc.robot.constants.ElevatorConstants;
import frc.robot.standards.ClosedLoopPosSubsystem;

import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.units.measure.Angle;


public class ElevatorSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
    // Private Variables
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ExiterIOInputsAutoLogged();

    private Angle setpoints;

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

  public void setPosition(Angle position) {
    setpoints = position;
    io.setPosition(position); //why not working?
  }

  public Angle getPosition() {
    return setpoints;
  }

  public boolean isFinished() {}
  
  public void zero() {} 

  //public void atPosition() {}
  
}
