package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final double speedShoot = -12.0;
  private static final double speedIntake = 12;
  private static final double speedIntakeFeeder = 3;
  private Timer shootElapsedTime = new Timer();
  public double shootingTime;
  private static final double launchDelay = 1.0;

  private double minAmpersTopBoundry = 40.00;
  private double minAmpersBottomBoundry = 12.00;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> {
              io.setTopVoltage(0.0);
              io.setBottomVoltage(0.0);
              io.setFeedVoltage(0.0);
            }));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    // double shootingTime = shootElapsedTime.get();
  }

  /** Returns a command that intakes a note and stops . */
  public Command intakeCommand() {
    return Commands.sequence(
        runOnce(
                () -> {
                  shootElapsedTime.reset();
                  shootElapsedTime.start();

                  io.setTopVoltage(speedIntake);
                  io.setBottomVoltage(speedIntake);
                  io.setFeedVoltage(speedIntakeFeeder);
                  // Commands.waitSeconds(0.5);

                  // After the timer starts sets the amper value to the top boundry
                  if (shootElapsedTime.hasElapsed(0.5)) {
                    minAmpersTopBoundry = inputs.feedCurrentAmps;
                  }
                })
            .alongWith(
                Commands.run(
                    () -> {
                      if (inputs.feedCurrentAmps >= minAmpersTopBoundry
                          && inputs.feedCurrentAmps < minAmpersBottomBoundry) {
                        io.setTopVoltage(0.0);
                        io.setBottomVoltage(0.0);
                        io.setFeedVoltage(0.0);
                      }
                    }))
            .finallyDo(
                () -> {
                  io.setTopVoltage(0.0);
                  io.setBottomVoltage(0);
                  io.setFeedVoltage(0.0);
                }));
  }

  /** Returns a command that launches a note. */
  public Command launchCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  io.setTopVoltage(speedShoot);
                  io.setBottomVoltage(speedShoot);
                  io.setFeedVoltage(speedShoot);
                }),
            Commands.waitSeconds(launchDelay),
            Commands.idle())
        .finallyDo(
            () -> {
              io.setTopVoltage(0.0);
              io.setBottomVoltage(0.0);
            });
  }
  /** Returns a command that launches a note. */
  public Command feederPushCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  io.setFeedVoltage(speedShoot);
                }),
            Commands.waitSeconds(launchDelay),
            Commands.idle())
        .finallyDo(
            () -> {
              io.setFeedVoltage(0.0);
            });
  }

  public Command launchAutoCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  io.setTopVoltage(speedShoot);
                  io.setFeedVoltage(speedShoot);
                }),
            Commands.waitSeconds(0.2)
                .andThen(
                    () -> {
                      io.setFeedVoltage(speedShoot);
                    }))
        .finallyDo(
            () -> {
              io.setTopVoltage(0.0);
              io.setBottomVoltage(0.0);
              io.setFeedVoltage(0.0);
            });
  }
}
