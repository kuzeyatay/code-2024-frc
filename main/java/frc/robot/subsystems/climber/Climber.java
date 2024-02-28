package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSet;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  // bottom arm is the arm that the shooter is connected to

  private static final Translation2d UrootPosition = new Translation2d(-0.12, 0.355);
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

  private static int climbExtend = 0;

  private static int climbIn = 0;

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kArmKp, kArmKi, kArmKd, ClimberConstants.kArmMotionConstraint);

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

        stowed = 0;

        climbIn = 30;

        climbExtend = 0;

        break;
      case SIM:
        kArmKp = 6.0;
        kArmKi = 0.0;
        kArmKd = 2.0;

        stowed = -60;

        climbIn = -50;

        climbExtend = -20;

        break;
      default:
        break;
    }
  }

  public Climber(ClimberIO io) {
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
    Logger.processInputs("Climber", inputs);

    boolean coast = DriverStation.isDisabled();
    if (coast != lastCoast) {
      lastCoast = coast;
      io.setBrakeMode(!coast, false);
    }

    // Get measured positions for top arm
    double angle = inputs.armInternalPositionRad - absoluteAngleOffset;
    UmechanismLigament.setAngle(new Rotation2d(angle));
    Logger.recordOutput("Mechanism2d/ClimbArm", Umechanism);
    Logger.recordOutput("Mechanism3d/ClimbMainArm", getPose3d(angle));
    Logger.recordOutput("ClimbMainArm/AngleRadians", angle);
    Logger.recordOutput("ClimbMainArm/AngleSetpointRadians", m_controller.getSetpoint().position);
    Logger.recordOutput("ClimbMainArm/AngleGoalRadians", m_controller.getGoal().position);

    // Reset when disabled
    if (DriverStation.isDisabled()) {

      io.setArmVoltage(0.0);
      m_controller.reset(angle);

      isRunning = false;

    } else {
      // Run controller when enabled
      if (isRunning && mode == 1) {
        m_controller.setGoal(Units.degreesToRadians(climbExtend));

      } else if (isRunning && mode == 2) {
        m_controller.setGoal(Units.degreesToRadians(climbIn));

      } else {
        m_controller.setGoal(Units.degreesToRadians(stowed));
      }
      io.setArmVoltage(m_controller.calculate(angle) + 0.005);
    }
  }

  /** Returns the 3D pose of the top arm for visualization. */
  private Pose3d getPose3d(double angle) {
    armPose =
        new Pose3d(UrootPosition.getX(), 0, UrootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
    return new Pose3d(
        UrootPosition.getX(), 0, UrootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
  }

  public Command extend() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 1;
        },
        () -> isRunning = false);
  }

  public Command runIn() {
    return startEnd(
        () -> {
          isRunning = true;
          mode = 2;
        },
        () -> isRunning = false);
  }
}
