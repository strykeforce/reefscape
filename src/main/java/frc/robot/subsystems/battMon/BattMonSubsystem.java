package frc.robot.subsystems.battMon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.battMon.BattMonIO.BattMonIOInputs;

public class BattMonSubsystem extends SubsystemBase {
  private BattMonIO io;
  private BattMonIOInputs inputs = new BattMonIOInputs();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
