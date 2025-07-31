// REBELLION 10014

package frc.robot.systems.intake.rollers;

import static frc.robot.systems.intake.rollers.RollersConstant.kMotorPort;
import static frc.robot.systems.intake.rollers.RollersConstant.kSensorPort;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.intake.rollers.RollersConstant.RollerHardware;
import frc.robot.systems.intake.rollers.RollersConstant.SensorHardware;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public enum RollerGoal {
    INTAKE(6.0),
    OUTPUT(-6.0),
    HOLD(1.0);

    private double intakeVolts;

    private RollerGoal(double intakeVolts) {
      this.intakeVolts = intakeVolts;
    }

    public double getGoalVolts() {
      return intakeVolts;
    }
  }

  // Hardware //
  private final SparkMax kMotor = new SparkMax(kMotorPort, MotorType.kBrushless);
  private CANrange kSensor = new CANrange(kSensorPort);
  private RelativeEncoder kEncoder = kMotor.getEncoder();

  // Records //
  private final RollerHardware kMotorHardware;
  private final SensorHardware kSensorHardware;

  // Configs //
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private CANrangeConfiguration sensorConfig = new CANrangeConfiguration();

  // Log the Goal //
  @AutoLogOutput(key = "Intake/Rollers/Goal")
  private RollerGoal goal = null;

  public Rollers() {
    this.kMotorHardware = RollersConstant.rollerHardware;
    this.kSensorHardware = RollersConstant.sensorHardware;

    motorConfig.inverted(kMotorHardware.invert());
    motorConfig.smartCurrentLimit(kMotorHardware.smartCurrentLimitAmps());
    motorConfig.secondaryCurrentLimit(kMotorHardware.secondaryCurrentLimitAmps());
    motorConfig.idleMode(kMotorHardware.idleMode());

    sensorConfig.ProximityParams.ProximityHysteresis = kSensorHardware.hysteresis();
    sensorConfig.ProximityParams.ProximityThreshold = kSensorHardware.threshold();
    sensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement =
        kSensorHardware.minSignalStrength();

    kMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kSensor.getConfigurator().apply(sensorConfig);
  }

  // Motor Setters //
  public void setVoltage(double volts) {
    volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
    kMotor.setVoltage(volts);
  }

  public void setGoal(RollerGoal desiredGoal) {
    goal = desiredGoal;
  }

  public void stop() {
    kMotor.stopMotor();
    goal = null;
  }

  // Motor Getters //
  public double getVelocityRotationsPerSec() {
    // TODO: Check if I need gearing
    return kEncoder.getVelocity() / 60.0;
  }

  public double getAppliedVoltage() {
    return kMotor.getAppliedOutput() * kMotor.getBusVoltage();
  }

  public double getOutputAmps() {
    return kMotor.getOutputCurrent();
  }

  public double getTemp() {
    return kMotor.getMotorTemperature();
  }

  // CANrange getters //
  public boolean hasGamepeice() {
    return kSensor.getIsDetected().getValue();
  }

  public double getSignalStrength() {
    return kSensor.getSignalStrength().getValue();
  }

  public double getDistance() {
    return kSensor.getDistance().getValueAsDouble();
  }

  @Override
  public void periodic() {

    // All the important values we need to log //
    Logger.recordOutput("Intake/Rollers/Temperature C*", getTemp());
    Logger.recordOutput("Intake/Rollers/Velocity Rot/s", getVelocityRotationsPerSec());
    Logger.recordOutput("Intake/Rollers/Voltage", getAppliedVoltage());
    Logger.recordOutput("Intake/Rollers/OutputAmps", getOutputAmps());

    Logger.recordOutput("Intake/Rollers/RangePeice", hasGamepeice());
    Logger.recordOutput("Intake/Rollers/RangeStrength", getSignalStrength());
    Logger.recordOutput("Intake/Rollers/RangeDistance", getDistance());

    if (goal != null) {
      setVoltage(goal.getGoalVolts());
    }
  }
}
