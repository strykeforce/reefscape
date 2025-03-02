package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class WaitForButtonPressCommand extends Command {
  private BooleanSupplier button;
  private Boolean initialState;

  public WaitForButtonPressCommand(BooleanSupplier button) {
    this.button = button;
  }

  @Override
  public void initialize() {
    initialState = button.getAsBoolean();
  }

  @Override
  public boolean isFinished() {
    return button.getAsBoolean();
  }
}
