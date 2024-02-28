package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TargetLock extends Command {

  private static final double DEADBAND = 0.1;
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(8, 8);

  private final Supplier<Double> TAG_TO_CHASE;
  private static Transform3d TAG_TO_GOAL =
      new Transform3d(
          new Translation3d(0.4, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)); //  0.0 meters
  // infront of it by default

  private final PhotonCamera photonCamera;
  private final Drive drive;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
  private DoubleSupplier xSupplier, ySupplier;

  private PhotonTrackedTarget lastTarget;

  public TargetLock(
      PhotonCamera photonCamera,
      Drive drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Double> TAG_TO_CHASE,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.photonCamera = photonCamera;
    this.drive = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.TAG_TO_CHASE = TAG_TO_CHASE;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {

    // Drive.ROTATION_PID_CONTROLLER.reset();
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());

    //  1.5 meters
    if (TAG_TO_CHASE.get() == 3 || TAG_TO_CHASE.get() == 8) {
      // this.cancel();
    }
  }

  @Override
  public void execute() {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command

    var robotPose2d = poseProvider.get();
    var robotPose3d =
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt =
          photonRes.getTargets().stream()
              .filter(t -> t.getFiducialId() == TAG_TO_CHASE.get())
              .filter(
                  t ->
                      !t.equals(lastTarget)
                          && t.getPoseAmbiguity() <= .2
                          && t.getPoseAmbiguity() != -1)
              .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose3d.transformBy(VisionConstants.kRobotToCam1);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }

    if (lastTarget == null) {

    } else {

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omegaSpeed,
              drive.getRotation()));
    }
  }

  @Override
  public void end(boolean interrupted) {

    Drive.ROTATION_PID_CONTROLLER.setP(Drive.ROTATION_KP);
  }
}
