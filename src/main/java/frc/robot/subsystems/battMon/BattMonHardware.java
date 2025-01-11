package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.constants.BattMonConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class BattMonHardware implements BattMonIO {

  private DigitalInput batt1 = new DigitalInput(0);
  private DigitalInput batt2 = new DigitalInput(1);
  private DigitalInput PDP = new DigitalInput(3);
  private DigitalInput temp = new DigitalInput(4);
  private DutyCycle batt1Cycle = new DutyCycle(batt1);
  private DutyCycle batt2Cycle = new DutyCycle(batt2);
  private DutyCycle PDPCycle = new DutyCycle(PDP);
  private DutyCycle tempCycle = new DutyCycle(temp);
  private Logger logger;

  private double batt1Output;
  private double batt2Output;
  private double pdpOutput;
  private double tempOutput;
  private Counter tempCounter = new Counter(temp);

  public BattMonHardware() {
    logger = LoggerFactory.getLogger(this.getClass());
    tempCounter.setUpSourceEdge(true, false);
  }

  @Override
  public void updateInputs(BattMonIOInputs inputs) {
    inputs.batt1Output = batt1Cycle.getOutput() * BattMonConstants.kBatt1;
    inputs.batt2Output = batt2Cycle.getOutput() * BattMonConstants.kBatt2;
    inputs.pdpOutput = PDPCycle.getOutput() * BattMonConstants.kPDP;
    inputs.tempOutput =
        (tempCycle.getHighTimeNanoseconds() / (tempCounter.getPeriod() / 1000000000))
            * BattMonConstants.kTemp;
  }
}
