package frc.robot.Vision;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.LimelightIO.Limelight;
import frc.robot.Vision.LimelightIO.LimelightIOReal;
import frc.robot.Vision.Photon.Photonvision;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class VisionSystem extends SubsystemBase {
  private static VisionSystem instance;
  private ShuffleboardTab driverTab;
  private ShuffleboardLayout layout;
  private Limelight limelight;

  private double photonTX;

  private double photonTrackedAprilId;
  public Photonvision photonCam = Photonvision.getInstance();
  public Photonvision falsePhotonCam;
  // SETS PHOTON CAMERA
  private boolean usingPhoton = true;
  // SETS Limelight
  private boolean usingLimelight = true;

  public static VisionSystem getInstance() {
    if (instance == null) {
      instance = new VisionSystem();
    }
    return instance;
  }

  private VisionSystem() {}

  /** Creates a new VisionSystem. */
  public VisionSystem(ShuffleboardTab driverTab) {
    this.driverTab = driverTab;
    if (usingLimelight) {
      this.limelight = new Limelight(new LimelightIOReal());

      this.layout =
          this.driverTab
              .getLayout("Limelight", BuiltInLayouts.kGrid)
              .withSize(4, 1)
              .withPosition(6, 1)
              .withProperties(
                  Map.of(
                      "Label position", "TOP",
                      "Number of columns", 5,
                      "Number of rows", 1));

      this.layout
          .addBoolean("Target Limelight", () -> this.limelight.validTarget())
          .withWidget(BuiltInWidgets.kBooleanBox);
      this.layout
          .addBoolean("Target Photon USB CAM", () -> this.photonCam.camTargets())
          .withWidget(BuiltInWidgets.kBooleanBox);

      // log tags
      Logger.recordOutput("Field/April Tags/April Tag 1", VisionConstants.AprilTag1);
      Logger.recordOutput("Field/April Tags/April Tag 2", VisionConstants.AprilTag2);
      Logger.recordOutput("Field/April Tags/April Tag 3", VisionConstants.AprilTag3);
      Logger.recordOutput("Field/April Tags/April Tag 4", VisionConstants.AprilTag4);
      Logger.recordOutput("Field/April Tags/April Tag 5", VisionConstants.AprilTag5);
      Logger.recordOutput("Field/April Tags/April Tag 6", VisionConstants.AprilTag6);
      Logger.recordOutput("Field/April Tags/April Tag 7", VisionConstants.AprilTag7);
      Logger.recordOutput("Field/April Tags/April Tag 8", VisionConstants.AprilTag8);
      Logger.recordOutput("Field/April Tags/April Tag 9", VisionConstants.AprilTag9);
      Logger.recordOutput("Field/April Tags/April Tag 10", VisionConstants.AprilTag10);
      Logger.recordOutput("Field/April Tags/April Tag 11", VisionConstants.AprilTag11);
      Logger.recordOutput("Field/April Tags/April Tag 12", VisionConstants.AprilTag12);
      Logger.recordOutput("Field/April Tags/April Tag 13", VisionConstants.AprilTag13);
      Logger.recordOutput("Field/April Tags/April Tag 14", VisionConstants.AprilTag14);
      Logger.recordOutput("Field/April Tags/April Tag 15", VisionConstants.AprilTag15);
      Logger.recordOutput("Field/April Tags/April Tag 16", VisionConstants.AprilTag16);
    }

    this.photonCam = Photonvision.getInstance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (usingPhoton) {
    // SmartDashboard.putBoolean("Using photon", getUsingPhoton());
    // if ( // DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == DriverStation.Alliance.Red&&
    // photonCam.getBestAprilTrackedTarget() == 4) {
    // RobotState.getInstance().setDriveControlState(DriveControlState.TARGET_LOCK);
    // } else if ( // DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue&&
    // photonCam.getBestAprilTrackedTarget() == 7) {
    // RobotState.getInstance().setDriveControlState(DriveControlState.TARGET_LOCK);
    // }
    // else RobotState.getInstance().setDriveControlState(DriveControlState.FIELD_RELATIVE);

    photonTrackedAprilId = (double) photonCam.getBestAprilTrackedTarget();

    // } else SmartDashboard.putBoolean("Using photon", false);

    if (usingLimelight) SmartDashboard.putBoolean("Using Limelight", getUsingLimelight());
    else SmartDashboard.putBoolean("Using Limelight", false);

    getPhotonAprilTagId();
  }

  public void setUsingPhoton(boolean usingPhoton) {
    this.usingPhoton = usingPhoton;
  }

  public boolean getUsingPhoton() {
    return usingPhoton;
  }

  public boolean getUsingLimelight() {
    return usingLimelight;
  }

  public Limelight getLimelight() {
    return limelight;
  }

  // FUCK YES I FUCKING LOVE PHOTONVISION

  public Double getPhotonAprilTagId() {

    photonTrackedAprilId = (double) photonCam.getBestAprilTrackedTarget();
    if (usingPhoton) {
      return photonTrackedAprilId;
    }

    return -1.0;
  }

  public Double getPhotonAprilTagIdForLock() {

    photonTrackedAprilId = (double) photonCam.getBestAprilTrackedTarget();
    if (usingPhoton && photonTrackedAprilId == 4) {
      return photonTrackedAprilId;
    }

    return 0.0;
  }

  public PhotonCamera getPhotonCam() {
    return photonCam.getCam();
  }

  public Photonvision getPhotonCamSystem() {
    return photonCam;
  }

  public Double getPhotonTargetX() {
    if (photonCam.getLatestResult().getBestTarget().getFiducialId() == 4) {
      photonTX = photonCam.getLatestResult().getBestTarget().getYaw();

      return photonTX;
    }
    return null;
  }
}
