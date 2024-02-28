package frc.robot.Vision.LimelightIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Vision.LimelightIO.LimelightIO.LimelightIOInputs;

public class Limelight {
  private final LimelightIO io;
  private final LimelightIOInputs inputs = new LimelightIOInputsAutoLogged();

  public Limelight(LimelightIO io) {
    this.io = io;
    initialize();
  }

  // enables leds by default
  public void initialize() {
    io.setCamMode(0);
    io.setLedMode(0);
  }

  /*
   * checks if there is a trusted target being detected by the limelight
   * getTlong() used to detect whether or not the limelight is using MegaTag
   */
  public boolean validTarget() {
    if (getTlong() > 200) {
      return inputs.tv > 0 && getTargetSpaceZ() < 4;
    }
    return inputs.tv > 0 && getTargetSpaceZ() < 2;
  }

  public double getTX() {
    // x translation
    return inputs.botPose[0];
  }

  public double getTY() {
    // y translation
    return inputs.botPose[1];
  }

  public double getTZ() {
    // z translation
    return inputs.botPose[2];
  }

  public double getRX() {
    // yaw
    return inputs.botPose[3];
  }

  public double getRY() {
    // pitch
    return inputs.botPose[4];
  }

  public double getRZ() {
    // roll
    return inputs.botPose[5];
  }

  public double getLatency() {
    // gets latency from limelight network tables (ms)
    return inputs.botPose[6] / 1000;
  }

  public double getTargetSpaceZ() {
    return inputs.targetSpace[2];
  }

  public double getTlong() {
    return inputs.tlong;
  }

  public Matrix<N3, N1> calculateTrust() {
    double[] vals = new double[3];
    for (int i = 0; i < vals.length; i++) {
      vals[i] = Math.pow(getTargetSpaceZ() / 2, 3);
    }
    return new Matrix<N3, N1>(N3.instance, N1.instance, vals);
  }

  // returns pose from limelight values in field space
  public Pose2d getPose2d() {
    return new Pose2d(getTX(), getTY(), Rotation2d.fromDegrees(getRZ()));
  }
}
