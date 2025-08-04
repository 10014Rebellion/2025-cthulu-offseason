package frc.robot.systems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.systems.intake.IntakeConstants.CANRange;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final CANrange mCANRange;
  private final SparkMax mRollerMotor;
  private final SparkMax mPivotMotor;
  private final AbsoluteEncoder mPivotEncoder;
  private final ProfiledPIDController mPivotController;
  private final ArmFeedforward mPivotFeedforward;

  private double goal = 0;

  private LoggedTunableNumber nTuneableP = new LoggedTunableNumber("Intake/Tuneables/kP", Pivot.kP);
  private LoggedTunableNumber nTuneableD = new LoggedTunableNumber("Intake/Tuneables/kD", Pivot.kD);
  private LoggedTunableNumber nTuneableVelocity =
      new LoggedTunableNumber("Intake/Tuneables/kVelocity", Pivot.kMaxVelocity);
  private LoggedTunableNumber nTuneableAccel =
      new LoggedTunableNumber("Intake/Tuneables/kAccel", Pivot.kMaxAcceleration);
  private LoggedTunableNumber nTuneableS = new LoggedTunableNumber("Intake/Tuneables/kS", Pivot.kS);
  private LoggedTunableNumber nTuneableV = new LoggedTunableNumber("Intake/Tuneables/kV", Pivot.kV);
  private LoggedTunableNumber nTuneableG = new LoggedTunableNumber("Intake/Tuneables/kG", Pivot.kG);
  private LoggedTunableNumber nTuneableSetpoint =
      new LoggedTunableNumber("Intake/Tuneables/Setpoint", 0.0);

  public Intake() {
    this.mCANRange = new CANrange(CANRange.kCANRangeID, Constants.kCTRECanBusName);
    this.mRollerMotor = new SparkMax(Roller.kMotorID, Roller.kMotorType);

    this.mPivotMotor = new SparkMax(Pivot.kMotorID, Pivot.kMotorType);
    this.mPivotEncoder = mPivotMotor.getAbsoluteEncoder();

    this.mPivotController =
        new ProfiledPIDController(
            Pivot.kP, 0, Pivot.kD, new Constraints(Pivot.kMaxVelocity, Pivot.kMaxAcceleration));
    this.mPivotController.setTolerance(Pivot.kTolerance);
    this.mPivotFeedforward = new ArmFeedforward(Pivot.kS, Pivot.kG, Pivot.kV, Pivot.kA);

    this.mRollerMotor.configure(
        Roller.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.mPivotMotor.configure(
        Pivot.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.mCANRange.getConfigurator().apply(CANRange.kRangeConfig);

    this.setDefaultCommand(intakePivotFF());
  }

  public boolean hasCoral() {
    return mCANRange.getIsDetected().getValue();
  }

  public void setRollerVolts(double pDesiredVolts) {
    mRollerMotor.setVoltage(clampVoltage(pDesiredVolts));
  }

  public void stopRollerMotor() {
    mRollerMotor.stopMotor();
  }

  public void stopPivotMotor() {
    mPivotMotor.stopMotor();
  }

  public void setPivotVolts(double pDesiredVolts) {
    if (isAtLimit(pDesiredVolts)) stopPivotMotor();
    else mPivotMotor.setVoltage(clampVoltage(pDesiredVolts));
  }

  private double clampVoltage(double pVolts) {
    return MathUtil.clamp(pVolts, -Constants.kRobotVoltage, Constants.kRobotVoltage);
  }

  private boolean isAtLimit(double pVolts) {
    return (pVolts < 0 && getPivotAngleDeg() <= Pivot.kLowerLimitDeg)
        || (pVolts > 0 && getPivotAngleDeg() >= Pivot.kUpperLimitDeg);
  }

  public FunctionalCommand setPivotVoltageCommand(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          mPivotMotor.setVoltage(clampVoltage(pVolts));
        },
        (interrupted) -> setPivotVolts(0),
        () -> false,
        this);
  }

  public FunctionalCommand setRollerVoltageCommand(double pVolts) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setRollerVolts(pVolts);
        },
        (interrupted) -> setRollerVolts(0),
        () -> false);
  }

  public FunctionalCommand intakePivotFF() {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
        },
        () -> {
          mPivotController.setGoal(getPivotAngleDeg());
          setPivotVolts(
              mPivotFeedforward.calculate(Units.degreesToRadians(getPivotAngleDeg()), 0.0));
        },
        (interrupted) ->
            setPivotVolts(
                mPivotFeedforward.calculate(Units.degreesToRadians(getPivotAngleDeg()), 0.0)),
        () -> false,
        this);
  }

  public FunctionalCommand intakePivotToGoal(double pGoalDegrees) {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
          mPivotController.setGoal(pGoalDegrees);
        },
        () ->
            setPivotVolts(
                mPivotController.calculate(getPivotAngleDeg())
                    + mPivotFeedforward.calculate(
                        Units.degreesToRadians(getPivotAngleDeg()),
                        Units.degreesToRadians(mPivotController.getSetpoint().velocity))),
        (interrupted) ->
            setPivotVolts(
                mPivotFeedforward.calculate(
                    Units.degreesToRadians(getPivotAngleDeg()),
                    Units.degreesToRadians(mPivotController.getSetpoint().velocity))),
        () -> mPivotController.atGoal(),
        this);
  }

  public FunctionalCommand intakeTunablePivotToGoal() {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
          mPivotController.setGoal(nTuneableSetpoint.get());
        },
        () -> {
          double PIDoutput = mPivotController.calculate(getPivotAngleDeg());
          double FFoutput =
              mPivotFeedforward.calculate(
                  Units.degreesToRadians(mPivotController.getSetpoint().position),
                  Units.degreesToRadians(mPivotController.getSetpoint().velocity));
          setPivotVolts(PIDoutput + FFoutput);
          Logger.recordOutput("Intake/Full Output", PIDoutput + FFoutput);
          Logger.recordOutput("Intake/PID Output", PIDoutput);
          Logger.recordOutput("Intake/FF Output", FFoutput);
        },
        (interrupted) -> setPivotVolts(0.0),
        () -> mPivotController.atGoal(),
        this);
  }

  public double getPivotAngleDeg() {
    if (mPivotEncoder.getPosition() >= 180.0) {
      return mPivotEncoder.getPosition() - 360;
    } else return mPivotEncoder.getPosition();
  }

  public double getPivotVoltage() {
    return mPivotMotor.getBusVoltage();
  }

  public double getRollersVoltage() {
    return mRollerMotor.getBusVoltage();
  }

  private void setPID(double kP, double kD) {
    mPivotController.setP(kP);
    mPivotController.setD(kD);
  }

  private void setConstraints(double kMaxVelo, double kMaxAccel) {
    mPivotController.setConstraints(new Constraints(kMaxVelo, kMaxAccel));
  }

  private void setFF(double kS, double kV, double kG) {
    mPivotFeedforward.setKg(kG);
    mPivotFeedforward.setKv(kV);
    mPivotFeedforward.setKs(kS);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Pivot/Voltage", getPivotVoltage());
    Logger.recordOutput("Intake/Pivot/Posistion", getPivotAngleDeg());
    Logger.recordOutput("Intake/Pivot/Goal", goal);

    Logger.recordOutput("Intake/Rollers/Voltage", getRollersVoltage());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setPID(nTuneableP.get(), nTuneableD.get());
        },
        nTuneableP,
        nTuneableD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setFF(nTuneableS.get(), nTuneableV.get(), nTuneableG.get());
        },
        nTuneableS,
        nTuneableV,
        nTuneableG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setConstraints(nTuneableVelocity.get(), nTuneableAccel.get());
        },
        nTuneableVelocity,
        nTuneableAccel);

    if (isAtLimit(getPivotVoltage())) stopPivotMotor();
  }
}
