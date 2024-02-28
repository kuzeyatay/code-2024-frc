package frc.robot.subsystems.intake;

import static frc.robot.util.SparkMax.CleanSparkMaxValue.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMax.SparkMaxBurnManager;
import frc.robot.util.SparkMax.SparkMaxPeriodicFrameConfig;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax rollerSparkMax;

  private RelativeEncoder armInternalEncoder;

  private double kArmGearRatio = (54.0 / 24.0) * (18.0 / 24.0) * (36 / 1);

  private final boolean armInvert;

  private final boolean rollerInvert;

  public IntakeIOSparkMax() {
    System.out.println("[Init] Creating intakeIOSparkMax");

    armSparkMax = new CANSparkMax(15, MotorType.kBrushless);
    rollerSparkMax = new CANSparkMax(16, MotorType.kBrushless);

    armInvert = false;
    rollerInvert = true;

    armInternalEncoder = armSparkMax.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      rollerSparkMax.restoreFactoryDefaults();
    }

    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    rollerSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(armSparkMax);
      SparkMaxPeriodicFrameConfig.configNotLeader(rollerSparkMax);

      // armInternalEncoder.setPosition(0.0);
      armInternalEncoder.setPositionConversionFactor(IntakeConstants.kPositionFactor);
      armInternalEncoder.setVelocityConversionFactor(IntakeConstants.kVelocityFactor);
      armInternalEncoder.setMeasurementPeriod(10);
      armInternalEncoder.setAverageDepth(2);

      armSparkMax.setInverted(armInvert);
      rollerSparkMax.setInverted(rollerInvert);

      armSparkMax.setSmartCurrentLimit(40);
      rollerSparkMax.setSmartCurrentLimit(40);

      armSparkMax.enableVoltageCompensation(12.0);
      rollerSparkMax.enableVoltageCompensation(12.0);
    }

    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    rollerSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      rollerSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.armInternalPositionRad =
        cleanSparkMaxValue(
            inputs.armInternalPositionRad,
            Units.rotationsToRadians(armInternalEncoder.getPosition()) / kArmGearRatio);
    inputs.armInternalVelocityRadPerSec =
        cleanSparkMaxValue(
            inputs.armInternalVelocityRadPerSec,
            Units.rotationsPerMinuteToRadiansPerSecond(armInternalEncoder.getPosition())
                / kArmGearRatio);
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armSparkMax.getMotorTemperature()};

    inputs.rollerAppliedVolts = rollerSparkMax.getAppliedOutput() * rollerSparkMax.getBusVoltage();
    inputs.rollerCurrentAmps = new double[] {rollerSparkMax.getOutputCurrent()};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerSparkMax.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean armBrake, boolean rollerBrake) {
    armSparkMax.setIdleMode(armBrake ? IdleMode.kBrake : IdleMode.kCoast);
    rollerSparkMax.setIdleMode(rollerBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
