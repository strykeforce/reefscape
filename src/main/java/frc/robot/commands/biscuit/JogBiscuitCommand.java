package frc.robot.commands.biscuit;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.biscuit.BiscuitSubsystem;

public class JogBiscuitCommand extends Command {

  private BiscuitSubsystem biscuitSubsystem;
  private Angle positionChange;

  public JogBiscuitCommand(BiscuitSubsystem biscuitSubsystem, Angle positionChange) {
    this.biscuitSubsystem = biscuitSubsystem;
    this.positionChange = positionChange;
    addRequirements(biscuitSubsystem);
  }

  @Override
  public void initialize() {
    biscuitSubsystem.setPosition(biscuitSubsystem.getPosition().plus(positionChange), false);
  }

  @Override
  public void execute() {
    biscuitSubsystem.setPosition(biscuitSubsystem.getPosition().plus(positionChange), false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
