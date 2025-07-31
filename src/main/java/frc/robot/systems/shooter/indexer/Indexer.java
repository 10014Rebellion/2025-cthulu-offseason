// REBELLION 10014

package frc.robot.systems.shooter.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.shooter.indexer.IndexerConstants.IndexerHardware;
import frc.robot.utils.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  public enum VelocityGoal {
    INTAKE(2000.0),
    OUTPUT(-2000.0),
    HOLD(1000.0);

    private double rollerRPM;

    private VelocityGoal(double rollerRPM) {
      this.rollerRPM = rollerRPM;
    }

    public double getGoalVelocityRPM() {
      return rollerRPM;
    }
  }

  // Hardware //
  private final SparkFlex kLeftMotor = new SparkFlex(IndexerConstants.kLeftMotorPort, MotorType.kBrushless);
  private RelativeEncoder kLeftEncoder;

  private final SparkFlex kRightMotor = new SparkFlex(IndexerConstants.kRightMotorPort, MotorType.kBrushless);
  private RelativeEncoder kRightEncoder;

  // Records //
  private final IndexerHardware kLeftMotorHardware;
  private final IndexerHardware kRightMotorHardware;

  // Configs //
  private SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig rightMotorConfig = new SparkFlexConfig();

  private LoggedTunableNumber kTuneableLeftP =
      new LoggedTunableNumber("Shooter/Left/Tuneables/kP", 1.0);
  private LoggedTunableNumber kTuneableLeftI =
      new LoggedTunableNumber("Shooter/Left/Tuneables/kI", 0.0);
  private LoggedTunableNumber kTuneableLeftD =
      new LoggedTunableNumber("Shooter/Left/Tuneables/kD", 0.0);

  private LoggedTunableNumber kTuneableRightP =
      new LoggedTunableNumber("Shooter/Right/Tuneables/kP", 1.0);
  private LoggedTunableNumber kTuneableRightI =
      new LoggedTunableNumber("Shooter/Right/Tuneables/kI", 0.0);
  private LoggedTunableNumber kTuneableRightD =
      new LoggedTunableNumber("Shooter/Right/Tuneables/kD", 0.0);

  // Log the Goal //
  @AutoLogOutput(key = "Intake/Rollers/Goal")
  private VelocityGoal goal = null;

  public Indexer() {
    this.kLeftMotorHardware = IndexerConstants.kIndexerHardware;

    leftMotorConfig.inverted(kLeftMotorHardware.invert());
    leftMotorConfig.smartCurrentLimit(kLeftMotorHardware.smartCurrentLimitAmps());
    leftMotorConfig.secondaryCurrentLimit(kLeftMotorHardware.secondaryCurrentLimitAmps());
    leftMotorConfig.idleMode(kLeftMotorHardware.idleMode());

    leftMotorConfig.encoder.positionConversionFactor(24.0/16.0);

    leftMotorConfig.closedLoop.p(kTuneableLeftP.get());
    leftMotorConfig.closedLoop.i(kTuneableLeftI.get());
    leftMotorConfig.closedLoop.d(kTuneableLeftD.get());

    kLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    kLeftEncoder = kLeftMotor.getEncoder();

    this.kRightMotorHardware = IndexerConstants.kIndexerHardware;

    rightMotorConfig.inverted(kRightMotorHardware.invert());
    rightMotorConfig.smartCurrentLimit(kRightMotorHardware.smartCurrentLimitAmps());
    rightMotorConfig.secondaryCurrentLimit(kRightMotorHardware.secondaryCurrentLimitAmps());
    rightMotorConfig.idleMode(kRightMotorHardware.idleMode());

    rightMotorConfig.encoder.positionConversionFactor(24.0/16.0);

    rightMotorConfig.closedLoop.p(kTuneableRightP.get());
    rightMotorConfig.closedLoop.i(kTuneableRightI.get());
    rightMotorConfig.closedLoop.d(kTuneableRightD.get());

    kRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kRightEncoder = kRightMotor.getEncoder();
  }

  // Motor Setters //
  public void setVoltage(double volts) {
    volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
    kLeftMotor.setVoltage(volts);
  }

  public void setVelocity(double leftRPM, double rightRPM) {
    kLeftMotor.getClosedLoopController().setReference(leftRPM, ControlType.kVelocity);
    kRightMotor.getClosedLoopController().setReference(leftRPM, ControlType.kVelocity);
  }

  public void setGoal(VelocityGoal desiredGoal) {
    goal = desiredGoal;
  }

  public void stop() {
    kLeftMotor.stopMotor();
    // goal = null;
  }

  // Motor Getters //
  public double getLeftVelocityRotationsPerSec() {
    // TODO: Check if I need gearing
    return kLeftEncoder.getVelocity() / 60.0;
  }

  public double getLeftAppliedVoltage() {
    return kLeftMotor.getAppliedOutput() * kLeftMotor.getBusVoltage();
  }

  public double getLeftOutputAmps() {
    return kLeftMotor.getOutputCurrent();
  }

  public double getLeftTemp() {
    return kLeftMotor.getMotorTemperature();
  }

  public double getRightVelocityRotationsPerSec() {
    // TODO: Check if I need gearing
    return kRightEncoder.getVelocity() / 60.0;
  }

  public double getRightAppliedVoltage() {
    return kRightMotor.getAppliedOutput() * kRightMotor.getBusVoltage();
  }

  public double getRightOutputAmps() {
    return kRightMotor.getOutputCurrent();
  }

  public double getRightTemp() {
    return kRightMotor.getMotorTemperature();
  }

  public void setLeftPID() {
    leftMotorConfig.closedLoop.p(kTuneableLeftP.get());
    leftMotorConfig.closedLoop.i(kTuneableLeftI.get());
    leftMotorConfig.closedLoop.d(kTuneableLeftD.get());
  }

  public void setRightPID() {
    rightMotorConfig.closedLoop.p(kTuneableRightP.get());
    rightMotorConfig.closedLoop.i(kTuneableRightI.get());
    rightMotorConfig.closedLoop.d(kTuneableRightD.get());
  }

  @Override
  public void periodic() {
    // All the important values we need to log //
    Logger.recordOutput("Shooter/RollerLeft/Temperature C*", getLeftTemp());
    Logger.recordOutput("Shooter/RollerLeft/Velocity Rot/s", getLeftVelocityRotationsPerSec());
    Logger.recordOutput("Shooter/RollerLeft/Voltage", getLeftAppliedVoltage());
    Logger.recordOutput("Shooter/RollerLeft/OutputAmps", getLeftOutputAmps());

    Logger.recordOutput("Shooter/RollerRight/Temperature C*", getRightTemp());
    Logger.recordOutput("Shooter/RollerRight/Velocity Rot/s", getRightVelocityRotationsPerSec());
    Logger.recordOutput("Shooter/RollerRight/Voltage", getRightAppliedVoltage());
    Logger.recordOutput("Shooter/RollerRight/OutputAmps", getRightOutputAmps());


    if (goal != null) {
      setVelocity(goal.getGoalVelocityRPM(), goal.getGoalVelocityRPM());
    }


    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      setLeftPID();
    }, kTuneableLeftP, kTuneableLeftI, kTuneableLeftD);
    
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      setRightPID();
    }, kTuneableRightP, kTuneableRightI, kTuneableRightD);
  }
}
