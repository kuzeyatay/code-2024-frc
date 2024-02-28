package frc.robot.Vision.Photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.ModeSet.Mode;
import frc.robot.Vision.VisionConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photonvision {
  private static Photonvision instance;
  private final PhotonCamera camera1;
  private final PhotonCamera camera2;
  private final PhotonPoseEstimator photonEstimator1;
  private final PhotonPoseEstimator photonEstimator2;
  private double lastEstTimestamp = 0;
  private int seenId;
  private int seenIdCam2;

  private double lastTX;

  // Simulation
  private PhotonCameraSim cameraSim1;
  private PhotonCameraSim cameraSim2;
  private VisionSystemSim visionSim;

  public static Photonvision getInstance() {
    if (instance == null) {
      instance = new Photonvision();
    }
    return instance;
  }

  private Photonvision() {
    camera1 = new PhotonCamera("USB_CAMERA");
    camera2 = new PhotonCamera("?");

    photonEstimator1 =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera1,
            VisionConstants.kRobotToCam1);
    photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimator2 =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera2,
            VisionConstants.kRobotToCam2);
    photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Mode.SIM != null) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.

      visionSim.addAprilTags(VisionConstants.kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim1 = new PhotonCameraSim(camera1, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim1, VisionConstants.kRobotToCam1);

      cameraSim1.enableDrawWireframe(true);

      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp2 = new SimCameraProperties();
      cameraProp2.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp2.setCalibError(0.35, 0.10);
      cameraProp2.setFPS(15);
      cameraProp2.setAvgLatencyMs(50);
      cameraProp2.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim2 = new PhotonCameraSim(camera2, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim2, VisionConstants.kRobotToCam2);

      cameraSim1.enableDrawWireframe(true);
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return camera1.getLatestResult();
  }

  public PhotonPipelineResult getCam2LatestResult() {
    return camera2.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator1.update();
    double latestTimestamp = camera1.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Mode.SIM != null) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam2() {
    var visionEst = photonEstimator2.update();
    double latestTimestamp = camera2.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Mode.SIM != null) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator1.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevsCam2(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator2.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  /**
   * @return if the photoncam detects a target or not
   */
  public boolean camTargets() {
    return getLatestResult().hasTargets();
  }

  public boolean cam2Targets() {
    return getCam2LatestResult().hasTargets();
  }

  /**
   * @return The best target that photoncam sees.
   */
  public PhotonTrackedTarget getPhotonTarget() {
    return getLatestResult().getBestTarget();
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public Pose2d getPose(Pose2d robotSimPose) {
    return robotSimPose;
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Mode.SIM != null) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!(Mode.SIM != null)) return null;
    return visionSim.getDebugField();
  }

  @AutoLogOutput(key = "Vision/Photon Tracked id cam 1")
  public int getBestAprilTrackedTarget() {

    if (camTargets()) {
      seenId = getLatestResult().getBestTarget().getFiducialId();
    }

    return seenId;
  }

  @AutoLogOutput(key = "Vision/Photon Tracked id cam 2")
  public int getBestAprilTrackedTargetcam2() {

    if (cam2Targets()) {
      seenIdCam2 = getCam2LatestResult().getBestTarget().getFiducialId();
    }

    return seenIdCam2;
  }

  public PhotonCamera getCam() {
    return camera1;
  }

  public PhotonCamera getCam2() {
    return camera2;
  }

  // target lock
  /**
   * @param id Id of the desired april tag
   * @return If the usb camera sees the april tag
   */
  public boolean validAprilTagTargetExists(int id) {
    var m_targets = camera2.getLatestResult().getTargets();
    for (PhotonTrackedTarget target : m_targets) {
      if (target.getFiducialId() == id) {
        return true;
      }
    }
    return false;
  }

  /** Whether the camera has a valid target */
  @AutoLogOutput(key = "Vision/Photon Cam 2 Has Target")
  public boolean getTV() {
    return camera2.getLatestResult().hasTargets();
  }

  /** Horizontal offset from crosshair to target (degrees) */
  @AutoLogOutput(key = "Vision/Photon Cam 2 tx")
  public double getTX() {
    if (validAprilTagTargetExists(7)) {
      var m_targets = camera2.getLatestResult().getTargets();
      for (PhotonTrackedTarget target : m_targets) {
        if (target.getFiducialId() == 7) {
          lastTX = target.getYaw();
        }
      }
    }
    return getTV() ? lastTX : Double.NaN;
  }

  public boolean validTargetExists() {
    return getTV();
  }
}
