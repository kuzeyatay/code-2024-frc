package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double armInternalPositionRad = 0.0;
    public double armInternalVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean armBrake, boolean rollerBrake) {}

  /** Run to setpoint angle in radians */
  default void runSetpoint(double setpointRads) {}
}
