// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision.LimelightIO;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final NetworkTableEntry camMode;
  private final NetworkTableEntry ledMode;
  private final NetworkTableEntry botPose;
  private final NetworkTableEntry targetSpace;

  private final NetworkTableEntry tv;
  private final NetworkTableEntry tlong;

  public LimelightIOReal() {
    camMode = table.getEntry("camMode");
    ledMode = table.getEntry("ledMode");
    tv = table.getEntry("tv");
    botPose = table.getEntry("botpose_wpiblue");
    targetSpace = table.getEntry("targetpose_robotspace");
    tlong = table.getEntry("tlong");
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.tv = tv.getDouble(0);
    inputs.botPose = botPose.getDoubleArray(new double[0]);
    inputs.targetSpace = targetSpace.getDoubleArray(new double[0]);
    inputs.tlong = tlong.getDouble(0);
  }

  @Override
  public void setCamMode(Number value) {
    camMode.setNumber(value);
  }

  @Override
  public void setLedMode(Number value) {
    ledMode.setNumber(value);
  }
}
