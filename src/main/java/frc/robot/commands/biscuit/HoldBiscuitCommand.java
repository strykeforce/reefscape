package frc.robot.commands.biscuit;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;

public class HoldBiscuitCommand extends InstantCommand {
  private BiscuitSubsystem biscuitSubsystem;

  public HoldBiscuitCommand(BiscuitSubsystem biscuitSubsystem) {
    this.biscuitSubsystem = biscuitSubsystem;
    addRequirements(biscuitSubsystem);
  }

  @Override
  public void initialize() {
    biscuitSubsystem.setPosition(biscuitSubsystem.getPosition());
  }
}
