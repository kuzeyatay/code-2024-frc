package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMax.SparkMaxBurnManager;
import frc.robot.util.SparkMax.SparkMaxPeriodicFrameConfig;

/**
 * This launcher implementation is for Spark Maxes driving NEO motors. For the Spark Flex/NEO
 * Vortex, replace all instances of "CANSparkMax" with "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;
  private final CANSparkMax feedMotor;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;
  private final RelativeEncoder feedEncoder;

  public ShooterIOSparkMax() {
    System.out.println("[Init] Creating ShooterIOSparkMax");

    topMotor = new CANSparkMax(10, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(11, MotorType.kBrushless);
    feedMotor = new CANSparkMax(12, MotorType.kBrushless);

    topEncoder = topMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    bottomEncoder = topMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    feedEncoder = feedMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(topMotor);
      SparkMaxPeriodicFrameConfig.configNotLeader(bottomMotor);

      topMotor.setSmartCurrentLimit(40);
      bottomMotor.setSmartCurrentLimit(40);
      feedMotor.setSmartCurrentLimit(30);
      feedMotor.setInverted(true);

      topMotor.enableVoltageCompensation(12.0);
      bottomMotor.enableVoltageCompensation(12.0);
      feedMotor.enableVoltageCompensation(12.0);
    }

    topMotor.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    bottomMotor.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    feedMotor.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      topMotor.burnFlash();
      bottomMotor.burnFlash();
      feedMotor.burnFlash();
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topPositionRad = Units.rotationsToRadians(topEncoder.getPosition());
    inputs.topVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity());
    inputs.topAppliedVolts = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
    inputs.topCurrentAmps = new double[] {topMotor.getOutputCurrent()};

    inputs.bottomPositionRad = Units.rotationsToRadians(bottomEncoder.getPosition());
    inputs.bottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity());
    inputs.bottomAppliedVolts = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();
    inputs.bottomCurrentAmps = new double[] {bottomMotor.getOutputCurrent()};

    inputs.feedPositionRad = Units.rotationsToRadians(feedEncoder.getPosition());
    inputs.feedVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(feedEncoder.getVelocity());
    inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
    inputs.feedCurrentAmps = feedMotor.getOutputCurrent();
  }

  @Override
  public void setTopVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomMotor.setVoltage(volts);
  }

  @Override
  public void setFeedVoltage(double volts) {
    feedMotor.setVoltage(volts);
  }
}
