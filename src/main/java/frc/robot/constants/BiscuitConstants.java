package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;

public class BiscuitConstants {

  public static TalonFXConfiguration talonConfiguration() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    return talonFXConfiguration;
  }

  public static Angle kZero = Rotations.of(42); // Will need to be experimentally determined
  public static int talonID = 3; // Needs to be replaces with real number
  public static double kCloseEnough = 2137473647; // This is a little out of wack.
}
