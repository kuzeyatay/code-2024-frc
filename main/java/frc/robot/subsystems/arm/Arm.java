package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSet;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public static Translation2d armOrigin =
      new Translation2d(-Units.inchesToMeters(9.11), Units.inchesToMeters(11.75));

  // bottom arm is the arm that the shooter is connected

  public record AimingParameters(
      Rotation2d driveHeading, Rotation2d armAngle, double driveFeedVelocity) {}

  private AimingParameters latestParameters = null;

  private static final Translation2d UrootPosition = new Translation2d(0.22, 0.38);
  private Mechanism2d Umechanism;
  private MechanismRoot2d UmechanismRoot;
  private MechanismLigament2d UmechanismLigament;

  private double absoluteAngleOffset = 0;
  private int mode = 0; // 1 for amp, 2 for chute, 3 for shoot from distance
  private boolean isRunning = false;
  private boolean lastCoast = false;

  private static double kArmKp = 0.0;
  private static double kArmKi = 0.0;
  private static double kArmKd = 0.0;

  public static Pose3d armPose;

  // SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
  private static int stowed = 0;

  // amp
  private static int amp = 0;

  // shoot from distance
  private static int shootDistance = 0;

  // intake from chute
  private static int chuteIntake = 0;

  private static int intakeGround = 0;

  private double currentAngle = 10;

  private double goal;

  // BOTOM POSITIONS ARE relative to the arm -no it isnt dumb fuck

  // bottom = arm which the shooter is connected to
  // top = main arm
  static {
    switch (ModeSet.currentMode) {
      case REAL:
        kArmKp = 10.0;
        kArmKi = 0.0;
        kArmKd = 0.0;
        // TODO CHANGE

        stowed = 10;

        amp = 0;

        shootDistance = 30;

        chuteIntake = 20;

        intakeGround = 60;

        break;
      case SIM:
        kArmKp = 6.0;
        kArmKi = 0.0;
        kArmKd = 2.0;

        stowed = -60;

        amp = -50;

        shootDistance = -20;

        chuteIntake = -110;

        intakeGround = -40;

        break;
      default:
        break;
    }
  }

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kArmKp, kArmKi, kArmKd, ArmConstants.kArmMotionConstraint);

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true, true);

    // create mechanisms pid
    m_controller.setP(kArmKp);
    m_controller.setD(kArmKd);

    // up
    Umechanism = new Mechanism2d(2, 5, new Color8Bit(Color.kBisque));
    UmechanismRoot =
        Umechanism.getRoot("ArmPivot", 2.0 + UrootPosition.getX(), UrootPosition.getY());

    UmechanismLigament =
        UmechanismRoot.append(
            new MechanismLigament2d("Arm Bottom", 0.2, -90, 4, new Color8Bit(Color.kGold)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // BmechanismLigament.setAngle(Units.radiansToDegrees(inputs.armBottomAbsolutePositionRad));

    // m_topController.setP(kArmKp);
    // m_bottomController.setP(kArmKp);

    // Update coast mode
    boolean coast = DriverStation.isDisabled();
    if (coast != lastCoast) {
      lastCoast = coast;
      io.setBrakeMode(!coast, false);
    }

    // Get measured positions for top arm
    double angle = inputs.armInternalPositionRad - absoluteAngleOffset;
    UmechanismLigament.setAngle(new Rotation2d(angle));
    Logger.recordOutput("Mechanism2d/Arm", Umechanism);
    Logger.recordOutput("Mechanism3d/MainArm", getPose3d(angle));
    Logger.recordOutput("MainArm/AngleRadians", angle);
    Logger.recordOutput("MainArm/AngleSetpointRadians", m_controller.getSetpoint().position);
    Logger.recordOutput("MainArm/AngleGoalRadians", m_controller.getGoal().position);

    // Reset when disabled
    if (DriverStation.isDisabled()) {

      io.setArmVoltage(0.0);
      m_controller.reset(angle);

      isRunning = false;

    } else {
      // Run controller when enabled
      if (isRunning && mode == 1) {
        m_controller.setGoal(Units.degreesToRadians(amp));

      } else if (isRunning && mode == 2) {
        m_controller.setGoal(Units.degreesToRadians(chuteIntake));

      } else if (isRunning && mode == 3) {
        m_controller.setGoal(Units.degreesToRadians(shootDistance));
        currentAngle = shootDistance;
      } else if (isRunning && mode == 4) {
        m_controller.setGoal(Units.degreesToRadians(intakeGround));

      } else if (isRunning && mode == 5) {
        m_controller.setGoal(goal);

      } else {
        m_controller.setGoal(Units.degreesToRadians(stowed));
      }
      io.setArmVoltage(m_controller.calculate(angle) + 0.005);
    }
    currentAngle = angle;
  }

  // check http://cs231n.stanford.edu/reports/2022/pdfs/121.pdf

  /** Returns the 3D pose of the top arm for visualization. */
  private Pose3d getPose3d(double angle) {
    armPose =
        new Pose3d(UrootPosition.getX(), 0, UrootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
    return new Pose3d(
        UrootPosition.getX(), 0, UrootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
  }

  @AutoLogOutput(key = "Arm/Mesured Angle")
  public double getAngle() {
    return Units.degreesToRadians(currentAngle);
  }

  public void setSpeed(Double speed) {
    io.setArmVoltage(speed);
  }

  public Command deliverAmp() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 1;
        },
        () -> isRunning = false);
  }

  public Command intake() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 4;
        },
        () -> isRunning = false);
  }

  public Command intakeChute() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 2;
        },
        () -> isRunning = false);
  }

  public Command shootFromDistance() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 3;
        },
        () -> isRunning = false);
  }

  public Command autoAim(double goal) {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 5;
          this.goal = goal;
        },
        () -> isRunning = false);
  }

  public Command Neutral() {
    return startEnd(
        () -> {
          mode = 0;
          isRunning = false;
        },
        () -> isRunning = true);
  }

  public Command autoAmp() {

    return Commands.sequence(
        runOnce(
            () -> {
              isRunning = true;
              mode = 1;
            }),
        Commands.waitSeconds(5)
            .finallyDo(
                () -> {
                  Commands.idle(this);
                }));
  }

  public Command autoIntake() {

    return Commands.sequence(
        runOnce(
            () -> {
              isRunning = true;
              mode = 4;
            }),
        Commands.waitSeconds(4)
            .finallyDo(
                () -> {
                  Commands.idle(this);
                }));
  }

  public Command autoShootFromDistance() {

    return Commands.sequence(
        runOnce(
            () -> {
              isRunning = true;
              mode = 3;
            }),
        Commands.waitSeconds(4)
            .finallyDo(
                () -> {
                  Commands.idle(this);
                }));
  }

  public AimingParameters getAimingParameters(Drive drive) {
    if (latestParameters != null) {
      // Cache previously calculated aiming parameters. Cache is invalidated whenever new
      // observations are added.
      return latestParameters;
    }

    Transform2d fieldToTarget =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
            .toTranslation2d()
            .toTransform2d();
    Pose2d fieldToPredictedVehicle = drive.getPredictedPose(0, 0);
    Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

    Translation2d predictedVehicleToTargetTranslation =
        fieldToPredictedVehicle.inverse().transformBy(fieldToTarget).getTranslation();
    Translation2d predictedVehicleFixedToTargetTranslation =
        fieldToPredictedVehicleFixed.inverse().transformBy(fieldToTarget).getTranslation();

    Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();

    Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
    double targetDistance = predictedVehicleToTargetTranslation.getNorm();

    double feedVelocity =
        drive.getSpeeds().vxMetersPerSecond * vehicleToGoalDirection.getSin() / targetDistance
            - drive.getSpeeds().vyMetersPerSecond
                * vehicleToGoalDirection.getCos()
                / targetDistance;

    latestParameters =
        new AimingParameters(
            targetVehicleDirection,
            new Rotation2d(
                targetDistance - armOrigin.getX(),
                FieldConstants.Speaker.centerSpeakerOpening.getZ() - armOrigin.getY() + 0.55),
            feedVelocity);
    Logger.recordOutput("RobotState/AimingParameters/Direction", latestParameters.driveHeading);
    Logger.recordOutput("RobotState/AimingParameters/ArmAngle", latestParameters.armAngle);
    Logger.recordOutput("RobotState/AimingParameters/DriveFeedVelocityRadPerS", feedVelocity);
    return latestParameters;
  }
}
