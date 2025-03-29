package frc.robot.subsystems.tagAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.BargeAlignConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class BargeAlignSubsystem extends MeasurableSubsystem {

  private DriveSubsystem driveSubsystem;

  private FlyskyJoystick flysky;
  private PIDController driveOmega;

  private BargeAlignStates curState = BargeAlignStates.FINISHED;
  private boolean isOnBlueSide = true;

  private double vX;
  private Rotation2d targetYaw;

  public BargeAlignSubsystem(FlyskyJoystick flysky, DriveSubsystem driveSubsystem) {
    this.flysky = flysky;
    this.driveSubsystem = driveSubsystem;
    this.driveOmega = new PIDController(6.0, 0, 0);
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
  }

  public void startBargeAlign() {
    if (isSafe()) {
      setState(BargeAlignStates.DRIVE);
      isOnBlueSide();
      driveOmega.reset();
    }
  }

  public void killBargeAlign() {
    setState(BargeAlignStates.FINISHED);
  }

  private void setState(BargeAlignStates desiredState) {
    this.curState = desiredState;
  }

  public BargeAlignStates getState() {
    return curState;
  }

  private boolean isOnBlueSide() {
    isOnBlueSide = driveSubsystem
            .getPoseMeters()
            .getX() < DriveConstants.kCenterLineX;
    vX = isOnBlueSide ? BargeAlignConstants.kXSpeed : -BargeAlignConstants.kXSpeed;
    targetYaw = isOnBlueSide ? BargeAlignConstants.kBlueDesiredYaw : BargeAlignConstants.kRedDesiredYaw;
    return isOnBlueSide;
  }

  private boolean isSafe() {
    double poseX = driveSubsystem.getPoseMeters().getX();

    return poseX > RobotStateConstants.kRedBargeSafeX
            || poseX < RobotStateConstants.kBlueBargeSafeX;
  }
  
  private boolean shouldRaiseElevator() {
    double poseX = driveSubsystem.getPoseMeters().getX();
    return isOnBlueSide ? poseX > BargeAlignConstants.kBlueRaiseElevatorX : poseX < BargeAlignConstants.kRedRaiseElevatorX;
  }
  
  private boolean shouldEjectAlgae() {
    double poseX = driveSubsystem.getPoseMeters().getX();
    return isOnBlueSide ? poseX > BargeAlignConstants.kBlueEjectAlgaeX : poseX < BargeAlignConstants.kRedEjectAlgaeX;
  }

  private double getYStickReading() {
    return flysky.getStr(); // just a placeholder, to remind me
  } // same joystick reading as the drive

  /*
   private void configureDriverBindings() {
      driveSubsystem.setDefaultCommand(
      new DriveTeleopCommand(
      () -> flysky.getStr()
      driveSubsystem,
      robotStateSubsystem));
  */

  private Rotation2d getTargetYaw() {
    return isOnBlueSide() ? BargeAlignConstants.kBlueDesiredYaw : BargeAlignConstants.kRedDesiredYaw;
  }

  @Override
  public void periodic() {
    switch (curState) {
      case DRIVE -> {
        double vOmega =
            driveOmega.calculate(
                driveSubsystem.getPoseMeters().getRotation().getRadians(), targetYaw.getRadians());
        driveSubsystem.move(vX, getYStickReading(), vOmega, true);
        if (shouldRaiseElevator()) {
          setState(BargeAlignStates.RAISE_ELEV);
        }
      }
      case RAISE_ELEV -> {
        double vOmega =
            driveOmega.calculate(
                driveSubsystem.getPoseMeters().getRotation().getRadians(), targetYaw.getRadians());
        driveSubsystem.move(vX, getYStickReading(), vOmega, true);
        if (shouldEjectAlgae()) {
          setState(BargeAlignStates.FINISHED);
        }
      }
      case FINISHED -> {}
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("BargeAlign/curState", () -> curState.ordinal()));
  }

  public enum BargeAlignStates {
    // INIT,
    DRIVE,
    RAISE_ELEV,
    FINISHED
  }
}
