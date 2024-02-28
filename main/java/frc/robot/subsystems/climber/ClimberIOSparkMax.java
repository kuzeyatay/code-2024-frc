package frc.robot.subsystems.climber;

import static frc.robot.util.SparkMax.CleanSparkMaxValue.cleanSparkMaxValue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMax.SparkMaxBurnManager;
import frc.robot.util.SparkMax.SparkMaxPeriodicFrameConfig;

public class ClimberIOSparkMax implements ClimberIO {
  private final CANSparkMax armSparkMax;
  private RelativeEncoder armInternalEncoder;
  private final boolean armInvert;

  private double kTopArmGearRatio = (64 / 1) * (64 / 22);

  public static final double kBottomSoftLimitReverse = 0.0;
  public static final double kBottomSoftLimitForward = Units.degreesToRadians(200);

  public ClimberIOSparkMax() {
    System.out.println("[Init] Creating ClimberIOSparkMax");

    armSparkMax = new CANSparkMax(19, MotorType.kBrushless);

    armInvert = true;

    armInternalEncoder = armSparkMax.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(armSparkMax);

      // armInternalEncoder.setPosition(0.0);
      armInternalEncoder.setPositionConversionFactor(ClimberConstants.kPositionFactor);
      armInternalEncoder.setVelocityConversionFactor(ClimberConstants.kVelocityFactor);
      armInternalEncoder.setMeasurementPeriod(10);
      armInternalEncoder.setAverageDepth(2);

      armSparkMax.setInverted(armInvert);

      armSparkMax.setSmartCurrentLimit(40);

      armSparkMax.enableVoltageCompensation(12.0);
    }

    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

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
