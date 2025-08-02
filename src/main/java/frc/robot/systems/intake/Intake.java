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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.systems.intake.IntakeConstants.CANRange;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final CANrange mCANRange;
  private final SparkMax mRollerMotor;
  private final SparkMax mPivotMotor;
  private final AbsoluteEncoder mPivotEncoder;
  private final ProfiledPIDController mPivotController;
  private final ArmFeedforward mPivotFeedforward;

  private double goal = 0;

  public Intake() {
    this.mCANRange = new CANrange(CANRange.kCANRangeID, Constants.kCTRECanBusName);
    this.mRollerMotor = new SparkMax(Roller.kMotorID, Roller.kMotorType);

    this.mPivotMotor = new SparkMax(Pivot.kMotorID, Pivot.kMotorType);
    this.mPivotEncoder = mPivotMotor.getAbsoluteEncoder();
    this.mPivotController =
        new ProfiledPIDController(
            Pivot.kP, 0, Pivot.kD, new Constraints(Pivot.kMaxVelocity, Pivot.kMaxAcceleration));
    this.mPivotController.setTolerance(1);
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
        || getPivotAngleDeg() >= Pivot.kUpperLimitDeg;
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
        () -> false,
        this);
  }

  public FunctionalCommand intakePivotFF() {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
        },
        () -> {
          mPivotController.setGoal(getPivotAngleDeg());
          setPivotVolts(
              mPivotFeedforward.calculate(
                  getPivotAngleDeg(), mPivotController.getSetpoint().velocity));
        },
        (interrupted) -> stopPivotMotor(),
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
                        getPivotAngleDeg(), mPivotController.getSetpoint().velocity)),
        (interrupted) -> stopPivotMotor(),
        () -> mPivotController.atGoal(),
        this);
  }

  public double getPivotAngleDeg() {
    return mPivotEncoder.getPosition() * Pivot.kEncoderConversionFactor;
  }

  public double getPivotVoltage() {
    return mPivotMotor.getBusVoltage();
  }

  public double getRollersVoltage() {
    return mRollerMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    if (isAtLimit(getPivotVoltage())) stopPivotMotor();

    Logger.recordOutput("Intake/Pivot/Voltage", getPivotVoltage());
    Logger.recordOutput("Intake/Pivot/Posistion", getPivotAngleDeg());
    Logger.recordOutput("Intake/Pivot/Goal", goal);

    Logger.recordOutput("Intake/Rollers/Voltage", getRollersVoltage());
  }
}
