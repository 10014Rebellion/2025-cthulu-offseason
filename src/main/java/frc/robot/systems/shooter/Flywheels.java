package frc.robot.systems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.systems.shooter.FlywheelConstants.bottomFlywheel;
import frc.robot.systems.shooter.FlywheelConstants.indexer;
import frc.robot.systems.shooter.FlywheelConstants.topFlywheel;
import frc.robot.systems.shooter.SingleFlywheel;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {

  private double mTrackedTopVelocity;
  private double mTrackedBottomVelocity;

  private final SparkFlex mIndexerMotor;

  private final SingleFlywheel mTopFlywheel;
  private final SingleFlywheel mBottomFlywheel;
  private final DigitalInput mAlgaeSensor;

  private LoggedTunableNumber nTuneableTopP =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/TopFlywheel/kP", FlywheelConstants.topFlywheel.kP);
  private LoggedTunableNumber nTuneableBottomP =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/BottomFlywheel/kP", FlywheelConstants.bottomFlywheel.kP);
  private LoggedTunableNumber nTuneableTopD =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/TopFlywheel/kD", FlywheelConstants.bottomFlywheel.kD);
  private LoggedTunableNumber nTuneableBottomD =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/BottomFlywheel/kD", FlywheelConstants.bottomFlywheel.kD);
  private LoggedTunableNumber nTuneableTopFF =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/TopFlywheel/kFF", FlywheelConstants.topFlywheel.kFF);
  private LoggedTunableNumber nTuneableBottomFF =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/BottomFlywheel/kFF", FlywheelConstants.bottomFlywheel.kFF);
  private LoggedTunableNumber nTuneableBottomRPM =
      new LoggedTunableNumber(
          "Flywheels/Tuneables/BottomFlywheel/RPM", 0.0);

  public Flywheels() {

    // this.mTopFF = topFlywheel.kFF;
    // this.mBottomFF = bottomFlywheel.kFF;

    // this.mTopFlywheelMotor = new SparkFlex(topFlywheel.kMotorID, MotorType.kBrushless);
    // this.mTopFlywheelMotor.configure(
    //     topFlywheel.kMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // this.mBottomFlywheelMotor = new SparkFlex(bottomFlywheel.kMotorID, MotorType.kBrushless);
    // this.mBottomFlywheelMotor.configure(
    //     bottomFlywheel.kMotorConfig,
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);

    // this.mBottomController = mTopFlywheelMotor.getClosedLoopController();
    // this.mTopController = mBottomFlywheelMotor.getClosedLoopController();

    mTopFlywheel = new SingleFlywheel(topFlywheel.kMotorID, topFlywheel.kMotorConfig);
    mBottomFlywheel = new SingleFlywheel(bottomFlywheel.kMotorID, bottomFlywheel.kMotorConfig);

    this.mIndexerMotor = new SparkFlex(indexer.kMotorID, MotorType.kBrushless);
    this.mIndexerMotor.configure(
        indexer.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.mAlgaeSensor = new DigitalInput(FlywheelConstants.indexer.kSensorPort);
  }

  private void setTopFlywheelVoltage(double pDesiredVolts) {
    if (pDesiredVolts != 0.0){
      mTopFlywheel.setVoltage(clampVoltage(pDesiredVolts));
    }
    else mTopFlywheel.stopMotor();
  }

  private void setBottomFlywheelVoltage(double pDesiredVolts) {
    if (pDesiredVolts != 0.0){
      mBottomFlywheel.setVoltage(clampVoltage(pDesiredVolts));
    }
    else mBottomFlywheel.stopMotor();
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
          setTopFlywheelVoltage(0);
          setBottomFlywheelVoltage(0);
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

  public FunctionalCommand intakeReefAlgaeCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setTopFlywheelVoltage(topFlywheel.Voltage.IntakeReef.getVoltage());
          setBottomFlywheelVoltage(bottomFlywheel.Voltage.IntakeReef.getVoltage());
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
    return mTopFlywheel.getVelocity();
  }

  public double getBottomFlywheelRPM() {
    return mBottomFlywheel.getVelocity();
  }

  public boolean getAlgaeDetected() {
    return !mAlgaeSensor.get();
  }

  public FunctionalCommand setTopFlywheelRPM(double pDesiredRPM) {
    return new FunctionalCommand(
        () -> {
          mTopFlywheel.setTargetVelocity(
              MathUtil.clamp(pDesiredRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
          System.out.println(
              "Clamping RPM to: "
                  + MathUtil.clamp(pDesiredRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
          if (pDesiredRPM > topFlywheel.kMaxRPM) {
            DriverStation.reportWarning(
                "SETPOINT RPM " + pDesiredRPM + "EXCEEDS THE MAXIMUM RPM OF " + topFlywheel.kMaxRPM,
                true);
          }
          mTrackedTopVelocity = pDesiredRPM;
        },
        () -> {
          Logger.recordOutput(
              "Flywheels/TopFlywheel/Output Voltage", mTopFlywheel.getAppliedOutput());
        },
        (interrupted) -> {},
        () -> false);
  }

  public FunctionalCommand setTuneableRPM() {
    return new FunctionalCommand(
        () -> {
          double pTopRPM = nTuneableBottomRPM.get();
          mTopFlywheel.setTargetVelocity(MathUtil.clamp(pTopRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
          double pBottomRPM = nTuneableBottomRPM.get() * FlywheelConstants.bottomMultipler;
          mBottomFlywheel.setTargetVelocity(MathUtil.clamp(pBottomRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
          mTrackedTopVelocity = pTopRPM;
          mTrackedBottomVelocity = pBottomRPM;
        },
        () -> {
        },
        (interrupted) -> {},
        () -> false);
  }

  public FunctionalCommand setBothFlywheelsRPM(double pDesiredRPM) {
    return new FunctionalCommand(
        () -> {
          mBottomFlywheel.setTargetVelocity(MathUtil.clamp(pDesiredRPM, -bottomFlywheel.kMaxRPM, bottomFlywheel.kMaxRPM));
          mTopFlywheel.setTargetVelocity(MathUtil.clamp(pDesiredRPM * FlywheelConstants.bottomMultipler, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));

          if (pDesiredRPM > bottomFlywheel.kMaxRPM * FlywheelConstants.bottomMultipler) {
            DriverStation.reportWarning(
                "SETPOINT RPM "
                    + pDesiredRPM
                    + "EXCEEDS THE MAXIMUM RPM OF "
                    + bottomFlywheel.kMaxRPM * FlywheelConstants.bottomMultipler,
                true);
            mTrackedBottomVelocity = pDesiredRPM;
          }

          if (pDesiredRPM > topFlywheel.kMaxRPM) {
            DriverStation.reportWarning(
                "SETPOINT RPM "
                    + pDesiredRPM
                    + "EXCEEDS THE MAXIMUM RPM OF "
                    + topFlywheel.kMaxRPM,
                true);
            mTrackedTopVelocity = pDesiredRPM;
          }
        },
        () -> {},
        (interrupted) -> {},
        () -> false);
  }

  public void setBothRPM(double pTopRPM, double pBottomRPM) {
    mTopFlywheel.setTargetVelocity(MathUtil.clamp(pTopRPM, -topFlywheel.kMaxRPM, topFlywheel.kMaxRPM));
    checkTopRPMLimits(pTopRPM);

    mBottomFlywheel.setTargetVelocity(MathUtil.clamp(pBottomRPM, -bottomFlywheel.kMaxRPM, bottomFlywheel.kMaxRPM));

    checkBottomRPMLimits(pBottomRPM);
    
  }

  private void checkTopRPMLimits(double pTopRPM) {
    if (pTopRPM > topFlywheel.kMaxRPM) {
      DriverStation.reportWarning(
          "SETPOINT RPM "
              + pTopRPM
              + "EXCEEDS THE MAXIMUM RPM OF "
              + topFlywheel.kMaxRPM,
          true);
    }
    mTrackedTopVelocity = pTopRPM;
  }

  private void checkBottomRPMLimits(double pBottomRPM) {
    if (pBottomRPM > bottomFlywheel.kMaxRPM) {
      DriverStation.reportWarning(
          "SETPOINT RPM "
              + pBottomRPM
              + "EXCEEDS THE MAXIMUM RPM OF "
              + bottomFlywheel.kMaxRPM * FlywheelConstants.bottomMultipler,
          true);
    }
    mTrackedBottomVelocity = pBottomRPM;
  }

  public FunctionalCommand setBottomFlywheelRPM(double pDesiredRPM) {
    return new FunctionalCommand(
        () -> {
          mBottomFlywheel.setTargetVelocity(
              MathUtil.clamp(pDesiredRPM, -bottomFlywheel.kMaxRPM, bottomFlywheel.kMaxRPM));
          if (pDesiredRPM > bottomFlywheel.kMaxRPM) {
            DriverStation.reportWarning(
                "SETPOINT RPM "
                    + pDesiredRPM
                    + "EXCEEDS THE MAXIMUM RPM OF "
                    + bottomFlywheel.kMaxRPM,
                true);
            mTrackedBottomVelocity = pDesiredRPM;
          }
        },
        () -> {},
        (interrupted) -> {},
        () -> false);
  }

  public boolean atSetpoint() {
    return mBottomFlywheel.atSetpoint() && mTopFlywheel.atSetpoint();
  }

  private double clampVoltage(double pVolts) {
    return MathUtil.clamp(pVolts, -Constants.kRobotVoltage, Constants.kRobotVoltage);
  }

  private void setPID(double topP, double bottomP, double topD, double bottomD) {
    mTopFlywheel.setPID(topP, topD);
    mBottomFlywheel.setPID(topP, topD);
  }

  private void setFF(double topFF, double bottomFF) {
    mTopFlywheel.setFF(topFF);
    mBottomFlywheel.setFF(bottomFF);
  }

  public void periodic() {
    Logger.recordOutput("Flywheels/TopFlywheel/Velocity", mTopFlywheel.getVelocity());
    Logger.recordOutput("Flywheels/TopFlywheel/Tracked Velocity", mTrackedTopVelocity);
    Logger.recordOutput("Flywheels/BottomFlywheel/Velocity", mBottomFlywheel.getVelocity());
    Logger.recordOutput("Flywheels/BottomFlywheel/Tracked Velocity", mTrackedBottomVelocity);
    Logger.recordOutput("Flywheels/Indexer/Velocity", mIndexerMotor.getEncoder().getVelocity());
    Logger.recordOutput("Flywheels/Indexer/Algae Detected", getAlgaeDetected());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setPID(
              nTuneableTopP.get(),
              nTuneableBottomP.get(),
              nTuneableTopD.get(),
              nTuneableBottomD.get());
        },
        nTuneableTopP,
        nTuneableBottomP,
        nTuneableTopD,
        nTuneableBottomD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setFF(nTuneableTopFF.get(), nTuneableBottomFF.get());
        },
        nTuneableTopFF,
        nTuneableBottomFF);
  }
}
