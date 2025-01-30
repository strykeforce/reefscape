package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class ElevatorConstants {
  // Setpoints
  // Idle
  public static final Angle kStowSetpoint = Rotations.of(0.0);
  public static final Angle kFunnelSetpoint = Rotations.of(0.0);
  public static final Angle kPrestageSetpoint = Rotations.of(0.0);

  // Algae removal
  public static final Angle kL2AlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kL3AlgaeSetpoint = Rotations.of(0.0);

  public static final Angle kL2AlgaeRemovalSetpoint = Rotations.of(0.0);
  public static final Angle kL3AlgaeRemovalSetpoint = Rotations.of(0.0);

  public static final Angle kSafeAlgaeRemovalSetpoint = Rotations.of(0.0);
  public static final Angle kSafeAlgaeRemovalRotateSetpoint = Rotations.of(0.0);

  // Coral score
  public static final Angle kL1CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL2CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL3CoralSetpoint = Rotations.of(0.0);
  public static final Angle kL4CoralSetpoint = Rotations.of(0.0);

  // Algae obtaining
  public static final Angle kFloorAlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kMicAlgaeSetpoint = Rotations.of(0.0);
  public static final Angle kHpAlgaeSetpoint = Rotations.of(0.0);

  // Algae scoring
  public static final Angle kProcessorSetpoint = Rotations.of(0.0);
  public static final Angle kBargeSetpoint = Rotations.of(0.0);
}
