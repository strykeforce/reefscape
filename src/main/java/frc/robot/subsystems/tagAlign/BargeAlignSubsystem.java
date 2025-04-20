package frc.robot.subsystems.tagAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.BargeAlignConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotStateConstants;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class BargeAlignSubsystem extends MeasurableSubsystem {

  private DriveSubsystem driveSubsystem;

  private FlyskyJoystick flysky;
  private PIDController driveOmega;
  private PIDController driveX;
  private Alliance alliance;

  private BargeAlignStates curState = BargeAlignStates.FINISHED;
  private boolean isOnBlueSide = true;

  private double vX;
  private Rotation2d targetYaw;
  private double targetX;

  public BargeAlignSubsystem(FlyskyJoystick flysky, DriveSubsystem driveSubsystem) {
    this.flysky = flysky;
    this.driveSubsystem = driveSubsystem;
    this.driveOmega = new PIDController(6.0, 0, 0);
    this.driveX = new PIDController(4, 0, 0);
    this.driveOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
  }

  public void startBargeAlign(Alliance alliance) {
    this.alliance = alliance;
    if (isSafe()) {
      this.isOnBlueSide = isOnBlueSide();
      setState(BargeAlignStates.DRIVE);
      setupBargeAlign();
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
    return driveSubsystem.getPoseMeters().getX() < DriveConstants.kCenterLineX;
  }

  private void setupBargeAlign() {
    vX = isOnBlueSide ? BargeAlignConstants.kXSpeed : -BargeAlignConstants.kXSpeed;
    targetYaw =
        isOnBlueSide ? BargeAlignConstants.kBlueDesiredYaw : BargeAlignConstants.kRedDesiredYaw;
    targetX =
        isOnBlueSide
            ? BargeAlignConstants.kBlueRaiseElevatorX + 0.5
            : BargeAlignConstants.kRedRaiseElevatorX - 0.5;
  }

  private boolean isSafe() {
    double poseX = driveSubsystem.getPoseMeters().getX();

    return poseX > RobotStateConstants.kRedBargeSafeX
        || poseX < RobotStateConstants.kBlueBargeSafeX;
  }

  private boolean shouldRaiseElevator() {
    double poseX = driveSubsystem.getPoseMeters().getX();
    return isOnBlueSide
        ? (poseX > BargeAlignConstants.kBlueRaiseElevatorX
            && poseX <= BargeAlignConstants.kBlueUnsafeX)
        : (poseX < BargeAlignConstants.kRedRaiseElevatorX
            && poseX >= BargeAlignConstants.kRedUnsafeX);
  }

  private boolean shouldDriveBackwards() {
    double poseX = driveSubsystem.getPoseMeters().getX();
    return isOnBlueSide
        ? poseX > BargeAlignConstants.kBlueUnsafeX
        : poseX < BargeAlignConstants.kRedUnsafeX;
  }

  private boolean shouldEjectAlgae() {
    double poseX = driveSubsystem.getPoseMeters().getX();
    return isOnBlueSide
        ? poseX > BargeAlignConstants.kBlueEjectAlgaeX
        : poseX < BargeAlignConstants.kRedEjectAlgaeX;
  }

  private double getYStickReading() {
    return flysky.getStr()
        * DriveConstants.kMaxSpeedMetersPerSecond
        * DriveConstants.kBargeScoreStickMultiplier
        * (alliance == Alliance.Blue ? -1 : 1); // just a placeholder, to remind me
  } // same joystick reading as the drive

  /*
   private void configureDriverBindings() {
      driveSubsystem.setDefaultCommand(
      new DriveTeleopCommand(
      () -> flysky.getStr()
      driveSubsystem,
      robotStateSubsystem));
  */

  public void terminate() {
    driveSubsystem.stopDriving();
    setState(BargeAlignStates.FINISHED);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("BargeAlign/State", curState);
    switch (curState) {
      case DRIVE -> {
        double driveXVel = driveX.calculate(driveSubsystem.getPoseMeters().getX(), targetX);
        double vOmega =
            driveOmega.calculate(
                driveSubsystem.getPoseMeters().getRotation().getRadians(), targetYaw.getRadians());
        driveSubsystem.move(driveXVel, getYStickReading(), vOmega, true);
        Logger.recordOutput("BargeAlign/XErr", driveX.getError());
        Logger.recordOutput("BargeAlign/Vx", driveXVel);
        Logger.recordOutput("BargeAlign/OmegaErr", driveOmega.getError());
        Logger.recordOutput("BargeAlign/Vomega", vOmega);
        if (shouldRaiseElevator()) {
          setState(BargeAlignStates.RAISE_ELEV);
        } else if (shouldDriveBackwards()) {
          setState(BargeAlignStates.REVERSE);
        }
      }
      case RAISE_ELEV -> {
        double vOmega =
            driveOmega.calculate(
                driveSubsystem.getPoseMeters().getRotation().getRadians(), targetYaw.getRadians());
        driveSubsystem.move(vX, getYStickReading(), vOmega, true);
        Logger.recordOutput("BargeAlign/Vx", vX);
        Logger.recordOutput("BargeAlign/OmegaErr", driveOmega.getError());
        Logger.recordOutput("BargeAlign/Vomega", vOmega);
        if (shouldEjectAlgae()) {
          terminate();
        }
      }
      case REVERSE -> {
        double vOmega =
            driveOmega.calculate(
                driveSubsystem.getPoseMeters().getRotation().getRadians(), targetYaw.getRadians());

        driveSubsystem.move(-vX, getYStickReading(), vOmega, true);

        Logger.recordOutput("BargeAlign/Vx", vX);
        Logger.recordOutput("BargeAlign/OmegaErr", driveOmega.getError());
        Logger.recordOutput("BargeAlign/Vomega", vOmega);

        if (shouldRaiseElevator()) {
          setState(BargeAlignStates.RAISE_ELEV);
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
    REVERSE,
    FINISHED
  }
}
