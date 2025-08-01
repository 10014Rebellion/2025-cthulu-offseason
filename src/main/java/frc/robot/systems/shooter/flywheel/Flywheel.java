// REBELLION 10014

package frc.robot.systems.shooter.flywheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.shooter.flywheel.FlywheelConstants.IndexerHardware;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static final LoggedTunableNumber kTuneableRPM = new LoggedTunableNumber("Shooter/RPM");

  public enum VelocityGoal {
    INTAKE(2000.0),
    OUTPUT(-2000.0),
    HOLD(1000.0),
    DEBUG(kTuneableRPM.get());

    private double rollerRPM;

    private VelocityGoal(double rollerRPM) {
      this.rollerRPM = rollerRPM;
    }

    public double getGoalVelocityRPM() {
      return rollerRPM;
    }
    // INDEXER: 34
  }

  // Hardware //
  private final SparkFlex kTopMotor =
      new SparkFlex(FlywheelConstants.kTopMotorPort, MotorType.kBrushless);
  private RelativeEncoder kTopEncoder;

  private final SparkFlex kBottomMotor =
      new SparkFlex(FlywheelConstants.kBottomMotorPort, MotorType.kBrushless);
  private RelativeEncoder kBottomEncoder;

  // Records //
  private final IndexerHardware kTopMotorHardware;
  private final IndexerHardware kBottomMotorHardware;

  // Configs //
  private SparkFlexConfig topMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();

  private LoggedTunableNumber kTuneabletopP =
      new LoggedTunableNumber("Shooter/top/Tuneables/kP", 1.0);
  private LoggedTunableNumber kTuneabletopI =
      new LoggedTunableNumber("Shooter/top/Tuneables/kI", 0.0);
  private LoggedTunableNumber kTuneabletopD =
      new LoggedTunableNumber("Shooter/top/Tuneables/kD", 0.0);

  private LoggedTunableNumber kTuneableBottomP =
      new LoggedTunableNumber("Shooter/Bottom/Tuneables/kP", 1.0);
  private LoggedTunableNumber kTuneableBottomI =
      new LoggedTunableNumber("Shooter/Bottom/Tuneables/kI", 0.0);
  private LoggedTunableNumber kTuneableBottomD =
      new LoggedTunableNumber("Shooter/Bottom/Tuneables/kD", 0.0);

  // Log the Goal //
  @AutoLogOutput(key = "Intake/Rollers/Goal")
  private VelocityGoal goal = null;

  public Flywheel() {
    this.kTopMotorHardware = FlywheelConstants.kIndexerHardware;

    topMotorConfig.inverted(kTopMotorHardware.invert());
    topMotorConfig.smartCurrentLimit(kTopMotorHardware.smartCurrentLimitAmps());
    topMotorConfig.secondaryCurrentLimit(kTopMotorHardware.secondaryCurrentLimitAmps());
    topMotorConfig.idleMode(kTopMotorHardware.idleMode());

    topMotorConfig.encoder.positionConversionFactor(24.0 / 16.0);

    topMotorConfig.closedLoop.p(kTuneabletopP.get());
    topMotorConfig.closedLoop.i(kTuneabletopI.get());
    topMotorConfig.closedLoop.d(kTuneabletopD.get());

    kTopMotor.configure(
        topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kTopEncoder = kTopMotor.getEncoder();

    this.kBottomMotorHardware = FlywheelConstants.kIndexerHardware;

    bottomMotorConfig.inverted(kBottomMotorHardware.invert());
    bottomMotorConfig.smartCurrentLimit(kBottomMotorHardware.smartCurrentLimitAmps());
    bottomMotorConfig.secondaryCurrentLimit(kBottomMotorHardware.secondaryCurrentLimitAmps());
    bottomMotorConfig.idleMode(kBottomMotorHardware.idleMode());

    bottomMotorConfig.encoder.positionConversionFactor(24.0 / 16.0);

    bottomMotorConfig.closedLoop.p(kTuneableBottomP.get());
    bottomMotorConfig.closedLoop.i(kTuneableBottomI.get());
    bottomMotorConfig.closedLoop.d(kTuneableBottomD.get());

    kBottomMotor.configure(
        bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kBottomEncoder = kBottomMotor.getEncoder();
  }

  // Motor Setters //
  public void setVoltage(double volts) {
    volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
    kTopMotor.setVoltage(volts);
  }

  public void setVelocity(double topRPM, double bottomRPM) {
    kTopMotor.getClosedLoopController().setReference(topRPM, ControlType.kVelocity);
    kBottomMotor.getClosedLoopController().setReference(topRPM, ControlType.kVelocity);
  }

  public void setGoal(VelocityGoal desiredGoal) {
    goal = desiredGoal;
  }

  public void stop() {
    kTopMotor.stopMotor();
    // goal = null;
  }

  // Motor Getters //
  public double gettopVelocityRotationsPerSec() {
    // TODO: Check if I need gearing
    return kTopEncoder.getVelocity() / 60.0;
  }

  public double gettopAppliedVoltage() {
    return kTopMotor.getAppliedOutput() * kTopMotor.getBusVoltage();
  }

  public double gettopOutputAmps() {
    return kTopMotor.getOutputCurrent();
  }

  public double gettopTemp() {
    return kTopMotor.getMotorTemperature();
  }

  public double getBottomVelocityRotationsPerSec() {
    // TODO: Check if I need gearing
    return kBottomEncoder.getVelocity() / 60.0;
  }

  public double getBottomAppliedVoltage() {
    return kBottomMotor.getAppliedOutput() * kBottomMotor.getBusVoltage();
  }

  public double getBottomOutputAmps() {
    return kBottomMotor.getOutputCurrent();
  }

  public double getBottomTemp() {
    return kBottomMotor.getMotorTemperature();
  }

  public void settopPID() {
    topMotorConfig.closedLoop.p(kTuneabletopP.get());
    topMotorConfig.closedLoop.i(kTuneabletopI.get());
    topMotorConfig.closedLoop.d(kTuneabletopD.get());
  }

  public void setBottomPID() {
    bottomMotorConfig.closedLoop.p(kTuneableBottomP.get());
    bottomMotorConfig.closedLoop.i(kTuneableBottomI.get());
    bottomMotorConfig.closedLoop.d(kTuneableBottomD.get());
  }

  @Override
  public void periodic() {
    // All the important values we need to log //
    Logger.recordOutput("Shooter/Rollertop/Temperature C*", gettopTemp());
    Logger.recordOutput("Shooter/Rollertop/Velocity Rot/s", gettopVelocityRotationsPerSec());
    Logger.recordOutput("Shooter/Rollertop/Voltage", gettopAppliedVoltage());
    Logger.recordOutput("Shooter/Rollertop/OutputAmps", gettopOutputAmps());

    Logger.recordOutput("Shooter/RollerBottom/Temperature C*", getBottomTemp());
    Logger.recordOutput("Shooter/RollerBottom/Velocity Rot/s", getBottomVelocityRotationsPerSec());
    Logger.recordOutput("Shooter/RollerBottom/Voltage", getBottomAppliedVoltage());
    Logger.recordOutput("Shooter/RollerBottom/OutputAmps", getBottomOutputAmps());

    if (goal != null) {
      setVelocity(goal.getGoalVelocityRPM(), goal.getGoalVelocityRPM());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          settopPID();
        },
        kTuneabletopP,
        kTuneabletopI,
        kTuneabletopD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setBottomPID();
        },
        kTuneableBottomP,
        kTuneableBottomI,
        kTuneableBottomD);
  }
}
