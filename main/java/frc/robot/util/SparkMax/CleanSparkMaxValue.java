package frc.robot.util.SparkMax;

// cleans spark max value for advantage scope
public class CleanSparkMaxValue {
  public static double cleanSparkMaxValue(double lastValue, double value) {
    if (Double.isNaN(value)
        || Double.isInfinite(value)
        || (Math.abs(value) < 1.0e-4 && Math.abs(lastValue) > 60.0)) {
      return lastValue;
    } else {
      return value;
    }
  }
}
