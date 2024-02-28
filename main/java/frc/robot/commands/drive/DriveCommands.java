package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class DriveCommands extends Command {
  private static final double DEADBAND = 0.1;
  private static DriveControlState m_currentDriveControlState;

  // infront of it by default

  public DriveCommands(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      PhotonCamera m_photonCamera,
      Supplier<Pose2d> m_poseProvider,
      Supplier<Double> m_TAG_TO_CHASE) {
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    addRequirements(drive);
  }

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    TARGET_LOCK
  }

  public static DriveControlState getDriveControlState() {
    return m_currentDriveControlState;
  }

  public static void setDriveControlState(DriveControlState driveControlState) {
    setCurrentDriveControlState(driveControlState);
  }

  private static void setCurrentDriveControlState(DriveControlState driveControlState) {
    m_currentDriveControlState = driveControlState;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  public static Command targetLockCommand(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, PhotonCamera cam) {

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  drive.getTargetLockRotation(),
                  drive.getRotation()));
        },
        drive);
  }

  public static Command alignWhileDrivingCommand(
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Translation2d> target,
      Drive drive) {
    PIDController pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(1.5);
    pid.enableContinuousInput(-180, 180);
    return new DeferredCommand(
        () ->
            new RepeatCommand(
                new FunctionalCommand(
                    () -> {
                      // Init
                    },
                    () -> {
                      Translation2d currentTranslation = drive.getPose().getTranslation();
                      Translation2d targetVector = currentTranslation.minus(target.get());
                      Rotation2d targetAngle = targetVector.getAngle();
                      double newSpeed;
                      // Apply deadband
                      double linearMagnitude =
                          MathUtil.applyDeadband(Math.hypot(xSpeed.get(), ySpeed.get()), DEADBAND);
                      Rotation2d linearDirection = new Rotation2d(xSpeed.get(), ySpeed.get());

                      // Square values
                      linearMagnitude = linearMagnitude * linearMagnitude;

                      // Calcaulate new linear velocity
                      Translation2d linearVelocity =
                          new Pose2d(new Translation2d(), linearDirection)
                              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                              .getTranslation();

                      // Convert to field relative speeds & send command
                      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                        newSpeed =
                            pid.calculate(
                                drive.getRotation().getDegrees(), targetAngle.getDegrees());
                      else
                        newSpeed =
                            pid.calculate(
                                drive.getRotation().getDegrees() + 180, targetAngle.getDegrees());
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                              newSpeed * drive.getMaxAngularSpeedRadPerSec(),
                              drive.getRotation()));
                    },
                    interrupted -> {
                      pid.close();
                    },
                    () -> {
                      return pid.atSetpoint();
                    },
                    drive)),
        Set.of(drive));
  }
}
