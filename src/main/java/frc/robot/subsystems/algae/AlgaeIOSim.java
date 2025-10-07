package frc.robot.subsystems.algae;

import frc.robot.constants.AlgaeConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO{
    //add code based off mechanical advantage
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private double appliedVoltage = 0.0;
    private AlgaeIOInputs inputs;
    public AlgaeIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }
  @Override
public void updateInputs(AlgaeIOInputs inputs) {
  if (DriverStation.isDisabled()) {
    setSpeed(0.0);
  }

  sim.update(AlgaeConstants.loopPeriodSecs);
  inputs.positionRad = sim.getAngularPositionRad();
  inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
  inputs.appliedVolts = appliedVoltage;
  inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  inputs.torqueCurrentAmps = gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
  inputs.tempCelsius = 0.0;     // Sim doesn't simulate temp
  inputs.tempFaulted = false;   // No real temp fault in sim
  inputs.isAlive = true;        // Sim always considered "alive"
}
public static AlgaeIOSim getInstance(){
  return sim;
}

  @Override
  public void setSpeed(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}