package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.constants.BattMonConstants;

public class BattMonHardware implements BattMonIO {

  private DigitalInput battVoltage = new DigitalInput(BattMonConstants.kBattVoltID);
  private DigitalInput battCurrent = new DigitalInput(BattMonConstants.kBattCurrentID);
  // private DigitalInput pdpVoltage = new DigitalInput(3);
  private DigitalInput temp = new DigitalInput(BattMonConstants.kBattTempID);
  private DutyCycle battVoltageCycle = new DutyCycle(battVoltage);
  private DutyCycle battCurrentCycle = new DutyCycle(battCurrent);
  // private DutyCycle pdpCycle = new DutyCycle(pdpVoltage);
  private DutyCycle tempCycle = new DutyCycle(temp);

  private Counter tempCounter = new Counter(temp);

  public BattMonHardware() {
    tempCounter.setUpSourceEdge(true, false);
  }

  @Override
  public void updateInputs(BattMonIOInputs inputs) {
    inputs.batteryVoltage =
        battVoltageCycle.getOutput() * BattMonConstants.kBattVoltSlope
            + BattMonConstants.kBattVoltOffset;
    inputs.batteryCurrent =
        battCurrentCycle.getOutput() * BattMonConstants.kBattCurrentSlope
            + BattMonConstants.kBattCurrentOffset;
    // inputs.pdpVoltage =
    //     pdpCycle.getOutput() * BattMonConstants.kPdpVoltSlope + BattMonConstants.kPdpVoltOffset;
    inputs.breakerTemp =
        ((tempCycle.getHighTimeNanoseconds() / (tempCounter.getPeriod() / 1000000000))
                - BattMonConstants.kBreakerTemp1)
            / BattMonConstants.kBreakerTemp2;
  }
}
