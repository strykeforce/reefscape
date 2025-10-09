package frc.robot.subsystems.algae;

import frc.robot.constants.AlgaeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
  private static final AlgaeIOSim INSTANCE = new AlgaeIOSim();

  private final DCMotor gearbox;
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  private AlgaeIOSim() {
    gearbox = DCMotor.getFalcon500(1); // Or whatever you use
    sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(gearbox, AlgaeConstants.moi, AlgaeConstants.gearRatio),
        gearbox);
  }

  public static AlgaeIOSim getInstance() {
    return INSTANCE;
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setSpeed(0.0);
    }

    sim.update(AlgaeConstants.loopPeriodSecs);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocity = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;
    inputs.tempFaulted = false;
    inputs.isAlive = true;
  }

  @Override
  public void setSpeed(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
  
  @Override
  public void update() {
    // Use the internal appliedVoltage for the simulation
    sim.setInput(appliedVoltage);
    sim.update(0.02); // Advance simulation by 20ms
}

  @Override
  public void setPct(double pct) {
    setSpeed(pct * 12.0);
  }

  @Override
  public void registerWith(org.strykeforce.telemetry.TelemetryService telemetryService) {
    // Optional: Add sim-specific telemetry if needed
  }
}
