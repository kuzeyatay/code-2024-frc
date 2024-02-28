package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/*
 * Class for auto alining and driving to april tags
 */

public class driveToApriltag extends Command {
  // check this yt video for better understanding of what this class does
  // https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=0s

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4, 4);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4, 4);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(8, 8);

  private final Supplier<Double> TAG_TO_CHASE;
  private static Transform3d TAG_TO_GOAL =
      new Transform3d(
          new Translation3d(0.4, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)); //  0.0 meters
  // infront of it by default

  private final PhotonCamera photonCamera;
  private final Drive drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

  /** Aligns */
  public driveToApriltag(
      PhotonCamera photonCamera,
      Drive drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Double> TAG_TO_CHASE) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.TAG_TO_CHASE = TAG_TO_CHASE;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    // amp
    if (TAG_TO_CHASE.get() == 5 || TAG_TO_CHASE.get() == 6) {
      TAG_TO_GOAL =
          new Transform3d(
              new Translation3d(0.42, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0)); //  0.30 meters
    }
    // chute
    else if (TAG_TO_CHASE.get() == 2 || TAG_TO_CHASE.get() == 9) {
      TAG_TO_GOAL =
          new Transform3d(
              new Translation3d(0.6, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0)); //  0.35 meters
    } else if (TAG_TO_CHASE.get() == 1 || TAG_TO_CHASE.get() == 10) {
      TAG_TO_GOAL =
          new Transform3d(
              new Translation3d(0.55, -0.1, 0.0), new Rotation3d(0.0, 0.0, 0)); //  0.5 meters
    }

    // speaker
    else if (TAG_TO_CHASE.get() == 7 || TAG_TO_CHASE.get() == 4) {
      TAG_TO_GOAL = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0));
    }
    //  1.5 meters
    else if (TAG_TO_CHASE.get() == 3 || TAG_TO_CHASE.get() == 8) {
      this.cancel();
    }
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose =
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
        var cameraPose = robotPose.transformBy(VisionConstants.kRobotToCam1);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }

    if (lastTarget == null) {

    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    }
  }
}
