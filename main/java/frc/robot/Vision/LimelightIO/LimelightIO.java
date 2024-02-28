// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision.LimelightIO;

import org.littletonrobotics.junction.AutoLog;

/** climber subsystem hardware interface. */
public interface LimelightIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class LimelightIOInputs {
    public double tv;
    public double[] botPose;
    public double[] targetSpace;
    public double tlong;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void setCamMode(Number value) {}

  public default void setLedMode(Number value) {}
}
