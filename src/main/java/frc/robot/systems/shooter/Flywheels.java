package frc.robot.systems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.systems.shooter.ShooterConstants.bottomFlywheel;
import frc.robot.systems.shooter.ShooterConstants.indexer;
import frc.robot.systems.shooter.ShooterConstants.topFlywheel;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final SparkFlex mTopFlywheelMotor;
  // private final RelativeEncoder mTopFlywheelEncoder;
  private final SparkFlex mBottomFlywheelMotor;
  // private final SparkFlex mBottomFlywheelEncoder;
  private final SparkFlex mIndexerMotor;

  public Flywheels() {
    this.mTopFlywheelMotor = new SparkFlex(topFlywheel.kMotorID, MotorType.kBrushless);
    this.mTopFlywheelMotor.configure(
        topFlywheel.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.mBottomFlywheelMotor = new SparkFlex(bottomFlywheel.kMotorID, MotorType.kBrushless);
    this.mBottomFlywheelMotor.configure(
        bottomFlywheel.kMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.mIndexerMotor = new SparkFlex(indexer.kMotorID, MotorType.kBrushless);
    this.mIndexerMotor.configure(
        indexer.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setTopFlywheelVoltage(double pDesiredVolts) {
    mTopFlywheelMotor.setVoltage(clampVoltage(pDesiredVolts));
  }

  private void setBottomFlywheelVoltage(double pDesiredVolts) {
    mBottomFlywheelMotor.setVoltage(clampVoltage(pDesiredVolts));
  }

  private void setIndexerVoltage(double pDesiredVolts) {
    mIndexerMotor.setVoltage(clampVoltage(pDesiredVolts));
  }

  public FunctionalCommand setTopFlywheelVoltageCommand(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setTopFlywheelVoltage(pVolts);
        },
        (interrupted) -> setTopFlywheelVoltage(0),
        () -> false);
  }

  public FunctionalCommand setBottomFlywheelVoltageCommand(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setBottomFlywheelVoltage(pVolts);
        },
        (interrupted) -> setBottomFlywheelVoltage(0),
        () -> false);
  }

  public FunctionalCommand setIndexerVoltageCommand(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setIndexerVoltage(pVolts);
        },
        (interrupted) -> setIndexerVoltage(0),
        () -> false);
  }

  private double clampVoltage(double pVolts) {
    return MathUtil.clamp(pVolts, -Constants.kRobotVoltage, Constants.kRobotVoltage);
  }

  public void periodic() {
    Logger.recordOutput(
        "Flywheels/TopFlywheel/Velocity", mTopFlywheelMotor.getEncoder().getVelocity());
    Logger.recordOutput(
        "Flywheels/BottomFlywheel/Velocity", mBottomFlywheelMotor.getEncoder().getVelocity());
    Logger.recordOutput("Flywheels/Indexer/Velocity", mIndexerMotor.getEncoder().getVelocity());
  }
}
