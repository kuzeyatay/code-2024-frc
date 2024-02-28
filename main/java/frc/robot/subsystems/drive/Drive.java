package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.Photon.Photonvision;
import frc.robot.Vision.VisionSystem;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.MixUtilities;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// all static methods are for the vision subsystem
public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(10.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(24.31);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(23.62);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private static Module[] modules = new Module[4]; // FL, FR, BL, BR

  public static SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private static Pose2d staticPose = new Pose2d();

  private Rotation2d lastGyroRotation = new Rotation2d();
  private Rotation2d currentRotation = new Rotation2d();

  // SIM Field on dashboard
  private Field2d field = new Field2d();

  // TARGET LOCK
  public static double ROTATION_KP = 1000.0;
  private static double ROTATION_KI = 0.0;
  private static double ROTATION_KD = 0.0;
  public static PIDController ROTATION_PID_CONTROLLER =
      new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
  private double TARGET_LOCK_FEED_FORWARD = 0.0001;

  // POSE ESTIMATION
  // The robot pose estimator for tracking swerve odometry and applying vision corrections.
  public Photonvision vision = Photonvision.getInstance();
  public SwerveDrivePoseEstimator poseEstimator;

  public Pose2d estimPose2D;
  public Pose2d estimPose2DCam2;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    // smart dashboard field
    // Set up custom logging to add the current path to a field 2d widget just in case
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

    // POSE ESTIMATION

    // Define the standard deviations for the pose estimator, which determine how fast the pose
    // estimate converges to the vision measurement. This should depend on the vision measurement
    // noise
    // and how many or how frequently vision measurements are applied to the pose estimator.
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, getRotation(), getPositions(), new Pose2d(), stateStdDevs, visionStdDevs);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      twist =
          new Twist2d(
              twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians());
      lastGyroRotation = gyroInputs.yawPosition;
    }
    // Apply the twist (change since last loop cycle) to the current pose
    pose = pose.exp(twist);
    staticPose = pose.exp(twist);

    updateRotation(getSpeeds().omegaRadiansPerSecond);

    // Dashboard
    field.setRobotPose(getPose());
    // Correct pose estimate with vision measurements
    if (VisionSystem.getInstance().getUsingPhoton()) {
      // Correct pose estimate with vision measurements
      var visionEst1 = vision.getEstimatedGlobalPose();

      visionEst1.ifPresent(
          est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevs(estPose);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });

      // Correct pose estimate with vision measurements
      var visionEst2 = vision.getEstimatedGlobalPoseCam2();

      visionEst2.ifPresent(
          est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevs(estPose);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }
  }

  public void simulationPeriodic() {
    if (VisionSystem.getInstance().getUsingPhoton()) {
      vision.simulationPeriodic(getStaticPose()); // this causes command loop
      // scheduler overrun error in simulation
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_LINEAR_SPEED);
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].runSetpoint(desiredStates[i]);
    }
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current static odometry pose. */
  public static Pose2d getStaticPose() {
    return staticPose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void updateRotation(double angularVelRps) {
    currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  // target locking for speaker
  public double getTargetLockRotation() {
    double tx = Photonvision.getInstance().getTX();
    if (!Photonvision.getInstance().validTargetExists()
        || MixUtilities.isWithinTolerance(tx, 0, 0.25)) {
      return 0;
    }

    // Increase kP based on horizontal velocity to reduce lag
    double vy = getSpeeds().vyMetersPerSecond; // Horizontal velocity
    double kp = ROTATION_KP;
    kp *= Math.max(1, vy * 1);
    ROTATION_PID_CONTROLLER.setP(kp);

    double rotation = ROTATION_PID_CONTROLLER.calculate(0, tx);
    double output = rotation + Math.copySign(TARGET_LOCK_FEED_FORWARD, rotation);

    if (!Photonvision.getInstance().validAprilTagTargetExists(7)) {
      return 0;
    }
    return output;
  }

  // Swerve drive pose estimation
  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /** Resets the current odometry pose to estimated pose seen by USB CAMERA. */
  public void setEstimPose1ToOdometry() {
    // Correct pose estimate with vision measurements
    if (VisionSystem.getInstance().getUsingPhoton() && pose != null) {
      var visionEst = vision.getEstimatedGlobalPose();
      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose;
            var estPose2d = estPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevs(estPose2d);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

            estimPose2D = estPose2d;
          });
      setPose(estimPose2D);
    }
    setPose(pose);
  }

  /** Resets the current odometry pose to estimated pose seen by second camera. */
  public void setEstimPose1ToOdometryCam2() {
    // Correct pose estimate with vision measurements
    if (VisionSystem.getInstance().getUsingPhoton() && pose != null) {
      var visionEst = vision.getEstimatedGlobalPoseCam2();
      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose;
            var estPose2d = estPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevsCam2(estPose2d);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

            estimPose2D = estPose2d;
          });
      setPose(estimPose2D);
    }
    setPose(pose);
  }

  /**
   * Returns the current 2D Photonvision Estimated pose of USB CAMERA.
   *
   * @return
   */
  @AutoLogOutput(key = "Odometry/2D Photon Estimated Robot Cam 1")
  public Pose2d get2DEstimatedPose() {
    // Correct pose estimate with vision measurements

    if (VisionSystem.getInstance().getUsingPhoton()) {
      var visionEst = vision.getEstimatedGlobalPose();

      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose;
            var estPose2d = estPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevs(estPose2d);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

            estimPose2D = estPose2d;
          });
      return estimPose2D;
    }
    return new Pose2d();

    // integrated photonvision here
  }

  /**
   * Returns the current 2D Photonvision Estimated pose of USB CAMERA.
   *
   * @return
   */
  @AutoLogOutput(key = "Odometry/2D Photon Estimated Robot Cam 2")
  public Pose2d get2DEstimatedPoseCam2() {
    // Correct pose estimate with vision measurements

    if (VisionSystem.getInstance().getUsingPhoton()) {
      var visionEst = vision.getEstimatedGlobalPoseCam2();

      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose;
            var estPose2d = estPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = vision.getEstimationStdDevsCam2(estPose2d);

            addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

            estimPose2DCam2 = estPose2d;
          });
      return estimPose2DCam2;
    }
    return new Pose2d();

    // integrated photonvision here
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    if (get2DEstimatedPose() != null) {
      return get2DEstimatedPose()
          .exp(
              new Twist2d(
                  getSpeeds().vxMetersPerSecond * translationLookaheadS,
                  getSpeeds().vyMetersPerSecond * translationLookaheadS,
                  getSpeeds().omegaRadiansPerSecond * rotationLookaheadS));
    }
    return new Pose2d();
  }
}
