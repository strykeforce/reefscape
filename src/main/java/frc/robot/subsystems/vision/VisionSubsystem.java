package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import WallEye.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {

  // Array of cameras
  WallEyeCam[] cams;

  // Array of camera positions
  Translation3d[] camPositions = {
    VisionConstants.kCam1Pose.getTranslation(),
    VisionConstants.kCam2Pose.getTranslation(),
    VisionConstants.kCam3Pose.getTranslation(),
    VisionConstants.kCam4Pose.getTranslation(),
    VisionConstants.kCam5Pose.getTranslation()
  };

  // Array of camera rotations
  Rotation3d[] camRotations = {
    VisionConstants.kCam1Pose.getRotation(),
    VisionConstants.kCam2Pose.getRotation(),
    VisionConstants.kCam3Pose.getRotation(),
    VisionConstants.kCam4Pose.getRotation(),
    VisionConstants.kCam5Pose.getRotation()
  };

  // Array of camera heights
  private double[] camHeights = {
    VisionConstants.kCam1Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam2Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam3Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam4Pose.getMeasureZ().in(Meters),
    VisionConstants.kCam5Pose.getMeasureZ().in(Meters)
  };

  // Array of camera names
  String[] camNames = {
    VisionConstants.kCam1Name,
    VisionConstants.kCam2Name,
    VisionConstants.kCam3Name,
    VisionConstants.kCam4Name,
    VisionConstants.kCam5Name
  };

  // Array of orange pi names
  String[] piNames = {
    VisionConstants.kPi1Name, VisionConstants.kPi2Name, VisionConstants.kPi3Name,
  };

  // Array of camera indexs
  int[] camIndex = {
    VisionConstants.kCam1Idx,
    VisionConstants.kCam2Idx,
    VisionConstants.kCam3Idx,
    VisionConstants.kCam4Idx,
    VisionConstants.kCam5Idx
  };

  private Swerve swerve = new Swerve();
  private DriveSubsystem driveSubsystem = new DriveSubsystem(swerve);
  /*Because we use two seperate loggers we can import one and then define the
  other here.*/
  private org.slf4j.Logger textLogger;
  private UdpSubscriber[] udpSubscriber;
  private AprilTagFieldLayout field;
  private boolean visionUpdating = true;
  private int minTags;
  private CircularBuffer<Double> gyroBuffer =
      new CircularBuffer<Double>(VisionConstants.kCircularBufferSize);
  private double timeSinceLastUpdate;
  private int updatesToWheels;
  private Matrix adaptiveMatrix;
  private ArrayList<Pair<WallEyeResult, Integer>> validResults = new ArrayList<>();
  private WallEyeTagResult[] lastResult;
  private Matrix<N3, N1> adativeMatrix;
  private Matrix<N3, N1> stdMatrix;
  private Logger logger;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    textLogger = LoggerFactory.getLogger("Vision");

    cams = new WallEyeCam[VisionConstants.kNumCams];
    udpSubscriber = new UdpSubscriber[VisionConstants.kNumPis];
    // We copy the matrix from our constants
    adaptiveMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
    // We then copy the adaptive matrix because they will differ later
    stdMatrix = adaptiveMatrix.copy();
    // I'm not sure why we put this in a try catch maybe could be removed
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      textLogger.error("BAD NEWS BEARS NO APRIL TAG LAYOUT");
    }
    // Fill our camera array
    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      cams[i] = new WallEyeCam(piNames[i], camIndex[i], -1);
    }
    // Initialize our udpSubscribers
    udpSubscriber[0] = new UdpSubscriber(0, cams[0], cams[1]);
    udpSubscriber[1] = new UdpSubscriber(1, cams[2], cams[3]);
    udpSubscriber[2] = new UdpSubscriber(2, cams[4]);
  }
  // Setter Methods
  public void setVisionUpdating(boolean updating) {
    this.visionUpdating = updating;
  }

  public void setMinTags(int minTags) {
    this.minTags = minTags;
  }
  // Getter Methods
  public boolean isVisionUpdating() {
    return visionUpdating;
  }

  public boolean cameraConnected(int index) {
    return cams[index].isCameraConnected();
  }

  private double getSeconds() {
    return RobotController.getFPGATime();
  }

  private double minTagDistance(WallEyePoseResult result) {

    Translation2d camLoc = result.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = result.getTagIDs();
    // This number's value doesn't really matter it just needs to be large
    double minDistance = 2767;

    // Go through the camera locations and finds the one closest to the tag
    for (int id : ids) {
      double dist =
          camLoc.getDistance(field.getTagPose(id).get().getTranslation().toTranslation2d());

      if (dist < minDistance) {
        minDistance = dist;
      }
    }
    return minDistance;
  }

  private double averageTagDistance(WallEyePoseResult result) {

    Translation2d camLoc = result.getCameraPose().getTranslation().toTranslation2d();
    int[] ids = result.getTagIDs();
    double totalDistance = 0.0;

    // Goes through the camera locations and gets the average distance
    for (int id : ids) {
      totalDistance +=
          camLoc.getDistance(field.getTagPose(id).get().getTranslation().toTranslation2d());
    }
    return totalDistance / ids.length;
  }

  // Filters
  private boolean camsAgreeWithWheels(Translation3d pose, WallEyeResult result) {

    ChassisSpeeds vel = driveSubsystem.getFieldRelSpeed();
    Pose2d curPose = driveSubsystem.getPoseMeters();

    Translation2d disp = (curPose.getTranslation().minus(pose.toTranslation2d()));

    double velMagnitude =
        Math.sqrt(Math.pow(vel.vxMetersPerSecond, 2) + Math.pow(vel.vyMetersPerSecond, 2));

    double dispMagnitude = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

    /*This gets our displacement and compares it to who much we could
    have moved.It does this by getting the velocity and plotting it on a
    graph. The graph will be in the docs.*/
    return result.getNumTags() >= minTags
        && dispMagnitude
            <= (velMagnitude * VisionConstants.kLinearCoeffOnVelFilter
                + VisionConstants.kOffsetOnVelFilter
                + Math.pow(velMagnitude * VisionConstants.kSquaredCoeffOnVelFilter, 2));
  }

  private boolean camsWithinField(Translation3d pose, WallEyePoseResult result) {
    return (result.getNumTags() >= 2 || result.getAmbiguity() < VisionConstants.kMaxAmbig)
        && pose.getMeasureX().in(Meters) < field.getFieldLength()
        && pose.getMeasureY().in(Meters) < field.getFieldWidth();
  }

  /*Large switch case to see get the standard deviation factor based on camera, how many
  tags we see and distance. An example of this graph will be in the docs.*/
  private double getStdDevFactor(double distance, int numTags, String camName) {
    switch (camName) {
      case "Upper Right":
      case "Upper Left":
        if (numTags == 1)
          return 1
              / VisionConstants.FOV58YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV58YUYVSingleTagCoeff * distance,
                      VisionConstants.FOV58YUYVPowerNumber));

        return 1
            / VisionConstants.FOV58YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58YUYVMultiTagCoeff * distance,
                    VisionConstants.FOV58YUYVPowerNumber));

      case "Rear":
        if (numTags == 1)
          return 1
              / VisionConstants.FOV58YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                      VisionConstants.FOV58YUYVPowerNumber));

        return 1
            / VisionConstants.FOV58YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV58MJPGSingleTagCoeff * distance,
                    VisionConstants.FOV58YUYVPowerNumber));

      case "Servo Left":
      case "Servo Right":
        if (numTags == 1)
          return 1
              / VisionConstants.FOV75YUYVBaseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.FOV75YUYVSingleTagCoeff * distance,
                      VisionConstants.FOV75YUYVPowerNumber));
        return 1
            / VisionConstants.FOV75YUYVBaseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.FOV75YUYVMultiTagCoeff * distance,
                    VisionConstants.FOV75YUYVPowerNumber));

      default:
        if (numTags == 1)
          return 1
              / VisionConstants.baseTrust
              * FastMath.pow(
                  VisionConstants.baseNumber,
                  FastMath.pow(
                      VisionConstants.singleTagCoeff * distance, VisionConstants.powerNumber));
        return 1
            / VisionConstants.baseTrust
            * FastMath.pow(
                VisionConstants.baseNumber,
                FastMath.pow(
                    VisionConstants.multiTagCoeff * distance, VisionConstants.powerNumber));
    }
  }

  private Pose3d getCloserPose(Pose3d pose1, Pose3d pose2, double rotation) {
    /*Which pose rotation is closer to our gyro. We subtract the absolute value of the gyro
    from the rotation of the pose and compare the two*/
    if (Math.abs(new Rotation2d(rotation).minus(pose1.getRotation().toRotation2d()).getRadians())
        <= Math.abs(
            new Rotation2d(rotation).minus(pose2.getRotation().toRotation2d()).getRadians()))
      return pose1;
    else return pose2;
  }

  private Pose3d getCorrectPose(Pose3d pose1, Pose3d pose2, double time, int camIndex) {
    double dist1 = Math.abs(camHeights[camIndex] - pose1.getZ());
    double dist2 = Math.abs(camHeights[camIndex] - pose2.getZ());
    // This filters out results by seeing if they are the height of the robot.
    if (dist1 < dist2 && dist1 < VisionConstants.kRobotHeight && dist1 > 0) {
      return pose1;
    }
    if (dist2 < dist1 && dist2 < VisionConstants.kRobotHeight && dist2 > 0) {
      return pose2;
    }
    // If we don't have enough data in the gyro buffer we default to returning a pose
    if (gyroBuffer.size() < VisionConstants.kCircularBufferSize) return pose1;

    // See what pose is closer the the gyro at the time of the photo's capture.
    double rotation =
        gyroBuffer.get(FastMath.floorToInt(((time / 1_000_000.0) / VisionConstants.kLoopTime)));
    return getCloserPose(pose1, pose2, rotation);
  }

  @Override
  public void periodic() {

    double gyroData = FastMath.normalizeMinusPiPi(driveSubsystem.getGyroRotation2d().getRadians());
    gyroBuffer.addFirst(gyroData);
    logger.recordOutput("Vision/Gyro Buffer", gyroData);

    if (getSeconds() - timeSinceLastUpdate > VisionConstants.kMaxTimeNoVision) {
      updatesToWheels = 0;
    }

    validResults.clear();

    for (int i = 0; i < VisionConstants.kNumCams; i++) {
      if (cams[i].hasNewUpdate()) {
        timeSinceLastUpdate = getSeconds();
        validResults.add(new Pair<WallEyeResult, Integer>(cams[i].getResults(), i));
      }
    }

    if (getSeconds() - timeSinceLastUpdate >= VisionConstants.kTimeToDecayDev) {
      // Decrease the thresholds required for a good pose over time. A graph is in the docs
      for (int i = 0; i < 2; i++) {
        double scaledWeight =
            VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                + VisionConstants.kStdDevDecayCoeff
                    * ((getSeconds() - timeSinceLastUpdate) - VisionConstants.kTimeToDecayDev);

        adaptiveMatrix.set(
            i,
            0,
            scaledWeight >= VisionConstants.kMinStdDev ? scaledWeight : VisionConstants.kMinStdDev);
      }
    }

    for (Pair<WallEyeResult, Integer> res : validResults) {
      if (res.getFirst() instanceof WallEyeResult) {
        /*Reset the adaptive matrix to a stricter value once we get a result.
        We set it to a stricter value then initially.*/
        adaptiveMatrix.set(0, 0, .1);
        adaptiveMatrix.set(1, 0, .1);

        WallEyePoseResult result = (WallEyePoseResult) res.getFirst();
        logger.recordOutput("Vision", result.getTimeStamp());
        int idx = res.getSecond();
        for (int i = 0; i < 2; i++) {
          stdMatrix.set(
              i,
              0,
              .1 / getStdDevFactor(minTagDistance(result), result.getNumTags(), camNames[idx]));
        }

        Pose3d cameraPose;
        Translation3d centerPose = new Translation3d();
        Rotation3d cameraRotation = new Rotation3d();

        if (result.getNumTags() > 1) {
          // If there our more then one tag in an image we can get pose possible pose
          cameraPose = result.getCameraPose();

          centerPose =
              cameraPose
                  .getTranslation()
                  .minus(camPositions[idx].rotateBy(cameraPose.getRotation()))
                  .rotateBy(camRotations[idx]);
        } else {
          // If there is one we get two possible poses and have to filter them.
          Pose3d cam1Pose = result.getFirstPose();
          Pose3d cam2Pose = result.getSecondPose();

          cam1Pose =
              new Pose3d(
                  cam1Pose.getTranslation(), cam1Pose.getRotation().rotateBy(camRotations[idx]));
          cam2Pose =
              new Pose3d(
                  cam2Pose.getTranslation(), cam2Pose.getRotation().rotateBy(camRotations[idx]));

          logger.recordOutput("Vision/Camera 1", cam1Pose);
          logger.recordOutput("Vision/Camera 2", cam2Pose);

          cameraPose = getCorrectPose(cam1Pose, cam2Pose, result.getTimeStamp(), idx);
          centerPose =
              cameraPose
                  .getTranslation()
                  .minus(camPositions[idx].rotateBy(cameraPose.getRotation()))
                  .rotateBy(camRotations[idx]);
          cameraRotation = cameraPose.getRotation();
        }
        if (camsWithinField(centerPose, result)) {
          // Is the pose in the field? If so, enjoy a updated position drive subsystem
          updatesToWheels++;
          logger.recordOutput("Vision/Accepted Cam", centerPose);
          // However we do have to be accepting the poses to use them
          if (visionUpdating) {
            driveSubsystem.addVisionMeasurement(
                new Pose2d(centerPose.toTranslation2d(), cameraRotation.toRotation2d()),
                result.getTimeStamp(),
                stdMatrix);
          }
        } else {
          logger.recordOutput("Vision/Rejected Cam", centerPose);
        }
      } else if (res.getFirst() instanceof WallEyeTagResult) {
        // I don't know what it does. I don't it does anything. But be careful about deletion.
        WallEyeTagResult tags = (WallEyeTagResult) res.getFirst();
        int idx = res.getSecond();
      }
    }
  }

  public WallEyeTagResult getLastResult(int index) {
    return lastResult[index];
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
