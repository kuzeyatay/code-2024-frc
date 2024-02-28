package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class MixUtilities {

  public static Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);

  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }
}
