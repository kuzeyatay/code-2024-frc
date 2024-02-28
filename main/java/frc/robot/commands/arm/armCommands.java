package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;

public class armCommands extends Command {

  public static Command alignWhileDrivingCommand(
      Supplier<Translation2d> target, Arm arm, Drive drive) {
    PIDController pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(1.5);
    // pid.enableContinuousInput(-180, 180);
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

                      // Convert to field relative speeds & send command
                      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                        newSpeed = pid.calculate(arm.getAngle(), targetAngle.getDegrees());
                      else newSpeed = pid.calculate(arm.getAngle(), targetAngle.getDegrees());

                      arm.setSpeed(newSpeed);
                    },
                    interrupted -> {
                      pid.close();
                    },
                    () -> {
                      return pid.atSetpoint();
                    })),
        Set.of(arm));
  }

  public static Command alignWhileDrivingCommand(Drive drive, Arm arm) {
    return new RepeatCommand(arm.autoAim(arm.getAimingParameters(drive).armAngle().getRadians()));
  }
}
