package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** climber subsystem hardware interface. */
public interface ClimberIO {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {

    public double armInternalPositionRad = 0.0;
    public double armInternalVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Set the top arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean armTopBrake, boolean armBottomBrake) {}
}
