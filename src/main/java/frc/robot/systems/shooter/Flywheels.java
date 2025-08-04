package frc.robot.systems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.systems.shooter.FlywheelConstants.bottomFlywheel;
import frc.robot.systems.shooter.FlywheelConstants.indexer;
import frc.robot.systems.shooter.FlywheelConstants.topFlywheel;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final ProfiledPIDController mTopController;
  private final ProfiledPIDController mBottomController;

  private final SparkFlex mTopFlywheelMotor;
  private final SparkFlex mBottomFlywheelMotor;
  private final SparkFlex mIndexerMotor;
  private final DigitalInput mAlgaeSensor;

  public Flywheels() {
    this.mTopController =
        new ProfiledPIDController(
            FlywheelConstants.topFlywheel.kP,
            0,
            FlywheelConstants.topFlywheel.kD,
            new Constraints(
                FlywheelConstants.topFlywheel.kMaxVelocity,
                FlywheelConstants.topFlywheel.kMaxAcceleration));
    mTopController.setTolerance(topFlywheel.kToleranceRPM);

    this.mBottomController =
        new ProfiledPIDController(
            FlywheelConstants.bottomFlywheel.kP,
            0,
            FlywheelConstants.bottomFlywheel.kD,
            new Constraints(
                FlywheelConstants.bottomFlywheel.kMaxVelocity,
                FlywheelConstants.bottomFlywheel.kMaxAcceleration));
    mBottomController.setTolerance(bottomFlywheel.kToleranceRPM);

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

    this.mAlgaeSensor = new DigitalInput(FlywheelConstants.indexer.kSensorPort);
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

  public FunctionalCommand setAllVoltageCommand(
      double topVolts, double bottomVolts, double indexerVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setTopFlywheelVoltage(topVolts);
          setBottomFlywheelVoltage(bottomVolts);
          setIndexerVoltage(indexerVolts);
        },
        (interrupted) -> {
          setIndexerVoltage(0);
        },
        () -> false);
  }

  public FunctionalCommand intakeAlgaeCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setTopFlywheelVoltage(topFlywheel.Voltage.IntakeAlgae.getVoltage());
          setBottomFlywheelVoltage(bottomFlywheel.Voltage.IntakeAlgae.getVoltage());
          setIndexerVoltage(indexer.Voltage.IndexAlgae.getVoltage());
        },
        (interrupted) -> {
          setIndexerVoltage(indexer.Voltage.HoldAlgae.getVoltage());
          setTopFlywheelVoltage(0.0);
          setBottomFlywheelVoltage(0.0);
        },
        () -> getAlgaeDetected());
  }

  public double getTopFlywheelRPM() {
    return mTopFlywheelMotor.getEncoder().getVelocity();
  }

  public double getBottomFlywheelRPM() {
    return mTopFlywheelMotor.getEncoder().getVelocity();
  }

  public boolean getAlgaeDetected() {
    return !mAlgaeSensor.get();
  }

  public FunctionalCommand setTopFlywheelRPM(double pDesiredRPM) {
    return new FunctionalCommand(
        () -> {
          mTopController.reset(getTopFlywheelRPM());
          mTopController.setGoal(
              MathUtil.clamp(pDesiredRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
          if (pDesiredRPM > topFlywheel.kMaxRPM) {
            DriverStation.reportWarning(
                "SETPOINT RPM " + pDesiredRPM + "EXCEEDS THE MAXIMUM RPM OF " + topFlywheel.kMaxRPM,
                true);
          }
        },
        () -> {
          setTopFlywheelVoltage(mTopController.calculate(getTopFlywheelRPM()));
        },
        (interrupted) -> {},
        () -> mTopController.atGoal(),
        this);
  }

  public FunctionalCommand setBottomFlywheelRPM(double pDesiredRPM) {
    return new FunctionalCommand(
        () -> {
          mBottomController.reset(getBottomFlywheelRPM());
          mBottomController.setGoal(
              MathUtil.clamp(pDesiredRPM, -bottomFlywheel.kMaxRPM, bottomFlywheel.kMaxRPM));
          if (pDesiredRPM > bottomFlywheel.kMaxRPM) {
            DriverStation.reportWarning(
                "SETPOINT RPM "
                    + pDesiredRPM
                    + "EXCEEDS THE MAXIMUM RPM OF "
                    + bottomFlywheel.kMaxRPM,
                true);
          }
        },
        () -> {
          setBottomFlywheelVoltage(mBottomController.calculate(getBottomFlywheelRPM()));
        },
        (interrupted) -> {},
        () -> mBottomController.atGoal(),
        this);
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
    Logger.recordOutput("Flywheels/Indexer/Algae Detected", getAlgaeDetected());
  }
}
