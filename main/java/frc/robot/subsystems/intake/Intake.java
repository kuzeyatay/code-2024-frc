package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final Translation2d rootPosition = new Translation2d(0.35, 0.2);
  private Mechanism2d mechanism;
  private MechanismRoot2d mechanismRoot;
  private MechanismLigament2d mechanismLigament;

  private double absoluteAngleOffset = 0;
  private boolean isRunning = false;
  private boolean lastCoast = false;

  private Supplier<Boolean> coastSupplier = () -> false;

  private static double neutralPositionDegrees = 0.0;
  private static double limboPositionDegrees = 0.0;
  private static double deployPositionDegrees = 0.0;
  private static double rollerVolts = 0.0;
  private static double kP = 0.0;
  private static double kD = 0.0;

  private ProfiledPIDController controller =
      new ProfiledPIDController(0.0, 0.0, 0.0, IntakeConstants.kArmMotionConstraint);

  static {
    switch (ModeSet.currentMode) {
      case REAL:
        neutralPositionDegrees = 0;

        deployPositionDegrees = 155;
        limboPositionDegrees = 20;
        rollerVolts = 7;
        kP = 10;
        kD = 0.0;
        break;
      case SIM:
        neutralPositionDegrees = 180;
        deployPositionDegrees = 40;
        rollerVolts = 8.0;
        kP = 6.00;
        kD = 2.0;

        break;
      default:
        break;
    }
  }

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    io.setBrakeMode(true, false);

    // Create mechanism
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Intake", 2.0 + rootPosition.getX(), rootPosition.getY());
    mechanismLigament =
        mechanismRoot.append(
            new MechanismLigament2d("IntakeArm", 0.35, 90, 4, new Color8Bit(Color.kLightGreen)));
    if (DriverStation.isAutonomous() || DriverStation.isAutonomousEnabled()) {
      io.setBrakeMode(false, false);
      isRunning = true;
    }

    controller.setP(kP);
    controller.setD(kD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update coast mode
    boolean coast = coastSupplier.get() && DriverStation.isDisabled();
    if (coast != lastCoast) {
      lastCoast = coast;
      io.setBrakeMode(!coast, false);
    }

    // Get measured positions
    double angle = inputs.armInternalPositionRad - absoluteAngleOffset;

    mechanismLigament.setAngle(new Rotation2d(angle));
    Logger.recordOutput("Mechanism2d/Intake", mechanism);
    Logger.recordOutput("Mechanism3d/Intake", getPose3d(angle));
    Logger.recordOutput("Intake/AngleRadians", angle);
    Logger.recordOutput("Intake/AngleSetpointRadians", controller.getSetpoint().position);
    Logger.recordOutput("Intake/AngleGoalRadians", controller.getGoal().position);
    Logger.recordOutput("Intake/Roller Speed", rollerVolts);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setArmVoltage(0.0);
      io.setRollerVoltage(0.0);
      controller.reset(angle);
      isRunning = false;

    } else {
      // Run controller when enabled
      if (isRunning) {
        controller.setGoal(Units.degreesToRadians(deployPositionDegrees));
        // Run controller and update motor output
        // io.setArmVoltage(controller.calculate(angle) + m_feedforward);
      } else {
        controller.setGoal(Units.degreesToRadians(limboPositionDegrees));
        // Run controller and update motor output
        // io.setArmVoltage(controller.calculate(angle) + m_feedforward);
      }
      io.setArmVoltage(controller.calculate(angle) + 0.005);
    }

    // Run roller
    io.setRollerVoltage(isRunning ? rollerVolts : 0.0);
  }

  /** Returns the 3D pose of the intake for visualization. */
  private Pose3d getPose3d(double angle) {
    return new Pose3d(
        rootPosition.getX(), 0.0, rootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  /** Command factory to extend and run the roller. */
  public Command runOutCommand() {
    return startEnd(
        () -> {
          isRunning = true;
        },
        () -> isRunning = false);
  }

  /** Command factory to intake and stop the roller. */
  public Command runInCommand() {
    return startEnd(
        () -> {
          isRunning = false;
        },
        () -> isRunning = true);
  }

  /** Command factory to extend and run the roller until intake command stops it */
  public Command runAutoOutCommand() {
    return runOnce(
        () -> {
          isRunning = true;
        });
  }

  public Command runAutoInCommand() {
    return runOnce(
        () -> {
          isRunning = false;
        });
  }
  // extends waits and intakes
  public Command inAndOut() {
    return Commands.sequence(
        runOnce(
            () -> {
              isRunning = true;
            }),
        Commands.waitSeconds(3)
            .finallyDo(
                () -> {
                  isRunning = false;
                }));
  }

  // extends waits and intakes
  public Command rollerForSecs() {
    return Commands.sequence(
        runOnce(
            () -> {
              io.setRollerVoltage(8);
            }),
        Commands.waitSeconds(0.)
            .finallyDo(
                () -> {
                  io.setRollerVoltage(0);
                }));
  }
}
