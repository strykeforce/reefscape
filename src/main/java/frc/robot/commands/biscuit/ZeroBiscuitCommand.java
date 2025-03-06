package frc.robot.commands.biscuit;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;

public class ZeroBiscuitCommand extends InstantCommand {
  private BiscuitSubsystem biscuitSubsystem;

  public ZeroBiscuitCommand(BiscuitSubsystem biscuitSubsystem) {
    this.biscuitSubsystem = biscuitSubsystem;
  }

  @Override
  public void initialize() {
    biscuitSubsystem.zero();
  }
}
