package frc.robot.subsystems.algae;

import frc.robot.constants.AlgaeConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO{
    //add code based off mechanical advantage
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private double appliedVoltage = 0.0;

    public AlgaeIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }
  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.data =
        new AlgaeIOData(
            sim.getAngularPositionRad(),
            sim.getAngularVelocityRadPerSec(),
            appliedVoltage,
            sim.getCurrentDrawAmps(),
            gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage),
            0.0,
            false,
            true);
  }
}
