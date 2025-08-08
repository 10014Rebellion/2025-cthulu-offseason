package frc.robot.systems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleFlywheel extends SubsystemBase {
  private SparkFlex motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController controller;
  private SparkFlexConfig motorConfig;
  private double targetVelo;

  public SingleFlywheel(int canID, SparkFlexConfig config) {
    motor = new SparkFlex(canID, MotorType.kBrushless);
    motorConfig = config;
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
  }

  public void setPID(double P, double D) {
    if (motor != null) {
      motorConfig.closedLoop.pid(P, 0, D);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void setFF(double FF) {
    if (motor != null) {
      motorConfig.closedLoop.velocityFF(FF);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void setTargetVelocity(double target) {
    targetVelo = target;
    controller.setReference(targetVelo, ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    return (Math.abs(targetVelo - getVelocity()) < FlywheelConstants.kTolerance && targetVelo != 0);
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setVoltage(double targetVoltage) {
    motor.setVoltage(targetVoltage);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getVoltage() {
    return motor.getBusVoltage();
  }
}
