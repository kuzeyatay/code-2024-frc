package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double topPositionRad = 0.0;
    public double topVelocityRadPerSec = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topCurrentAmps = new double[] {};

    public double bottomPositionRad = 0.0;
    public double bottomVelocityRadPerSec = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomCurrentAmps = new double[] {};

    public double feedPositionRad = 0.0;
    public double feedVelocityRadPerSec = 0.0;
    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the launcher wheel at the specified voltage. */
  public default void setTopVoltage(double volts) {}

  /** Run the launcher wheel at the specified voltage. */
  public default void setBottomVoltage(double volts) {}

  /** Run the feeder wheel at the specified voltage. */
  public default void setFeedVoltage(double volts) {}
}
