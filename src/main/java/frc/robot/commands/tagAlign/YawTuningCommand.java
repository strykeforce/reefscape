package frc.robot.commands.tagAlign;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class YawTuningCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private Pose2d start;
  private ProfiledPIDController driveOmega;
  private boolean adjustYaw = false;
  private DoubleSupplier pSupplier;

  public YawTuningCommand(DriveSubsystem driveSubsystem, DoubleSupplier pSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.pSupplier = pSupplier;

    this.driveOmega = new ProfiledPIDController(8.0, 0, 0, new Constraints(1000, 1000));
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    this.driveOmega.setP(pSupplier.getAsDouble());
    Logger.recordOutput("TagAlignSubsystem/OmegaKp", driveOmega.getP());
    start = driveSubsystem.getPoseMeters();
    adjustYaw = false;
    driveSubsystem.move(0.5, 0, 0, true);
  }

  @Override
  public void execute() {
    if (!adjustYaw
        && driveSubsystem.getPoseMeters().getTranslation().getDistance(start.getTranslation())
            > 1) {
      adjustYaw = true;
      driveOmega.reset(driveSubsystem.getGyroRotation2d().getRadians());
    }
    if (adjustYaw) {
      double vOmega =
          driveOmega.calculate(
              driveSubsystem.getGyroRotation2d().getRadians(), Math.toRadians(100));
      Logger.recordOutput("TagAlignSubsystem/DriveOmegaError", driveOmega.getPositionError());
      Logger.recordOutput("TagAlignSubsystem/OmegaSetpoint", driveOmega.getSetpoint().position);
      driveSubsystem.move(0.5, 0, vOmega, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDriving();
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.getPoseMeters().getTranslation().getDistance(start.getTranslation()) > 4;
  }
}
