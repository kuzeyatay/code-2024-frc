package frc.robot.subsystems.arm;

import static frc.robot.util.SparkMax.CleanSparkMaxValue.cleanSparkMaxValue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMax.SparkMaxBurnManager;
import frc.robot.util.SparkMax.SparkMaxPeriodicFrameConfig;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax armSparkMaxFollower;
  private RelativeEncoder armInternalEncoder;
  private final boolean armInvert;

  private double kTopArmGearRatio = (20.0 / 1.0) * (4 / 1);

  public static final double kBottomSoftLimitReverse = 0.0;
  public static final double kBottomSoftLimitForward = Units.degreesToRadians(200);

  public ArmIOSparkMax() {
    System.out.println("[Init] Creating ArmIOSparkMax");

    armSparkMax = new CANSparkMax(17, MotorType.kBrushless);
    armSparkMaxFollower = new CANSparkMax(18, MotorType.kBrushless);

    armInvert = true;

    armInternalEncoder = armSparkMax.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(armSparkMax);
      SparkMaxPeriodicFrameConfig.configLeaderFollower(armSparkMaxFollower);

      // armInternalEncoder.setPosition(0.0);
      armInternalEncoder.setPositionConversionFactor(ArmConstants.kPositionFactor);
      armInternalEncoder.setVelocityConversionFactor(ArmConstants.kVelocityFactor);
      armInternalEncoder.setMeasurementPeriod(10);
      armInternalEncoder.setAverageDepth(2);

      armSparkMax.setInverted(armInvert);
      armSparkMaxFollower.follow(armSparkMax, true);

      armSparkMax.setSmartCurrentLimit(40);
      armSparkMaxFollower.setSmartCurrentLimit(40);

      armSparkMax.enableVoltageCompensation(12.0);
    }

    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    armSparkMaxFollower.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      armSparkMaxFollower.burnFlash();
    }
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.armInternalPositionRad =
        cleanSparkMaxValue(
            inputs.armInternalPositionRad,
            Units.rotationsToRadians(armInternalEncoder.getPosition()) / kTopArmGearRatio);
    inputs.armInternalVelocityRadPerSec =
        cleanSparkMaxValue(
            inputs.armInternalVelocityRadPerSec,
            Units.rotationsPerMinuteToRadiansPerSecond(armInternalEncoder.getPosition())
                / kTopArmGearRatio);
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armSparkMax.getMotorTemperature()};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean armBrake, boolean rollerBrake) {
    armSparkMax.setIdleMode(armBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
