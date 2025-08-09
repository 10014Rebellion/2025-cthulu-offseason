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
  private double CGoffset = Pivot.kCGAngleOffset;

  private LoggedTunableNumber nTuneableP = new LoggedTunableNumber("Intake/Tuneables/kP", Pivot.kP0);
  private LoggedTunableNumber nTuneableD = new LoggedTunableNumber("Intake/Tuneables/kD", Pivot.kD0);
  private LoggedTunableNumber nTuneableVelocity =
      new LoggedTunableNumber("Intake/Tuneables/kVelocity", Pivot.kMV0);
  private LoggedTunableNumber nTuneableAccel =
      new LoggedTunableNumber("Intake/Tuneables/kAccel", Pivot.kMA0);
  private LoggedTunableNumber nTuneableS = new LoggedTunableNumber("Intake/Tuneables/kS", Pivot.kS0);
  private LoggedTunableNumber nTuneableV = new LoggedTunableNumber("Intake/Tuneables/kV", Pivot.kV0);
  private LoggedTunableNumber nTuneableG = new LoggedTunableNumber("Intake/Tuneables/kG", Pivot.kG0);
  private LoggedTunableNumber nTuneableSetpoint =
      new LoggedTunableNumber("Intake/Tuneables/Setpoint", 0.0);

  private int slot = 0;

  public Intake() {
    this.mCANRange = new CANrange(CANRange.kCANRangeID, Constants.kCTRECanBusName);
    this.mRollerMotor = new SparkMax(Roller.kMotorID, Roller.kMotorType);

    this.mPivotMotor = new SparkMax(Pivot.kMotorID, Pivot.kMotorType);
    this.mPivotEncoder = mPivotMotor.getAbsoluteEncoder();

    this.mPivotController =
        new ProfiledPIDController(
            Pivot.kP0, 0, Pivot.kD0, new Constraints(Pivot.kMV0, Pivot.kMA0));
    this.mPivotController.setTolerance(Pivot.kTolerance);
    this.mPivotFeedforward = new ArmFeedforward(Pivot.kS0, Pivot.kG0, Pivot.kV0, Pivot.kA0);

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
        },
        () -> {
          double FFoutput = mPivotFeedforward.calculate(Units.degreesToRadians(getPivotAngleDeg() + CGoffset), 0.0);
          setPivotVolts(FFoutput);
              Logger.recordOutput("Intake/Pivot/FF Output", FFoutput);
        },
        (interrupted) ->
            setPivotVolts(
                mPivotFeedforward.calculate(Units.degreesToRadians(getPivotAngleDeg() + CGoffset), 0.0)),
        () -> false,
        this);
  }

  public FunctionalCommand setIntakePivotCmd(double pGoalDegrees) {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
          mPivotController.setGoal(pGoalDegrees);
        },
        () ->
            setPivotVolts(
                mPivotController.calculate(getPivotAngleDeg())
                    + mPivotFeedforward.calculate(
                        Units.degreesToRadians(mPivotController.getSetpoint().position + CGoffset),
                        Units.degreesToRadians(mPivotController.getSetpoint().velocity))),
        (interrupted) ->
            setPivotVolts(
                mPivotFeedforward.calculate(
                    Units.degreesToRadians(getPivotAngleDeg() + CGoffset),
                    0.0)),
        () -> mPivotController.atGoal(),
        this);
  }

  public FunctionalCommand setUnendingIntakePivotCmd(double pGoalDegrees) {
    return new FunctionalCommand(
        () -> {
          mPivotController.reset(getPivotAngleDeg());
          mPivotController.setGoal(pGoalDegrees);
        },
        () -> {

        double PIDoutput = mPivotController.calculate(getPivotAngleDeg());
        double FFoutput =
            mPivotFeedforward.calculate(
                Units.degreesToRadians(mPivotController.getSetpoint().position + CGoffset),
                Units.degreesToRadians(mPivotController.getSetpoint().velocity));
        setPivotVolts(PIDoutput + FFoutput);
        Logger.recordOutput("Intake/Pivot/Full Output", PIDoutput + FFoutput);
        Logger.recordOutput("Intake/Pivot/PID Output", PIDoutput);
        Logger.recordOutput("Intake/Pivot/FF Output", FFoutput);},
        (interrupted) ->
            setPivotVolts(
                mPivotFeedforward.calculate(
                    Units.degreesToRadians(getPivotAngleDeg() + CGoffset),
                    0.0)),
        () -> false,
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
                  Units.degreesToRadians(mPivotController.getSetpoint().position + CGoffset),
                  Units.degreesToRadians(mPivotController.getSetpoint().velocity));
          setPivotVolts(PIDoutput + FFoutput);
          Logger.recordOutput("Intake/Pivot/Full Output", PIDoutput + FFoutput);
          Logger.recordOutput("Intake/Pivot/PID Output", PIDoutput);
          Logger.recordOutput("Intake/Pivot/FF Output", FFoutput);
          Logger.recordOutput("Intake/Pivot/Intake Position Setpoint Deg", mPivotController.getSetpoint().position);
          Logger.recordOutput("Intake/Pivot/Intake Position Setpoint Radian", Units.degreesToRadians(mPivotController.getSetpoint().position));
          Logger.recordOutput("Intake/Pivot/Intake Velocity Setpoint Deg", mPivotController.getSetpoint().velocity);
          Logger.recordOutput("Intake/Pivot/Intake Velocity Setpoint Radian", Units.degreesToRadians(mPivotController.getSetpoint().velocity));
        },
        (interrupted) -> setPivotVolts(0.0),
        () -> false,
        this);
  }

  public FunctionalCommand intakeCoralCmd() {
    return new FunctionalCommand(
        () -> {},
        () -> setRollerVolts(Roller.Voltage.IntakeCoral.getVoltage()),
        (interrupted) -> setRollerVolts(Roller.Voltage.HoldCoral.getVoltage()),
        () -> hasCoral());
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

  private void setPIDandFF(double kP, double kD, double kMaxVelocity, double kMaxAccel, double kS, double kV, double kG) {
    mPivotController.setP(kP);
    mPivotController.setD(kD);
    mPivotController.setConstraints(
      new Constraints(kMaxVelocity, kMaxAccel)
    );
    mPivotFeedforward.setKg(kG);
    mPivotFeedforward.setKv(kV);
    mPivotFeedforward.setKs(kS);
  }

  private void setSlot(int slot) {
    if (slot > 1) {
      System.out.println("NOTE! THIS DOES NOT CURRENTLY EXIST!");
    }
    else this.slot = slot;
  }

  public FunctionalCommand scoreCoralCmd() {
    return new FunctionalCommand(
        () -> {},
        () -> setRollerVolts(Roller.Voltage.ScoreL1.getVoltage()),
        (interrupted) -> {
          if (!hasCoral()) setRollerVolts(0.0);
          else setRollerVolts(Roller.Voltage.SlowScoreL1.getVoltage());
        },
        () -> !hasCoral());
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Pivot/Voltage", getPivotVoltage());
    Logger.recordOutput("Intake/Pivot/AppliedOutput", mPivotMotor.getAppliedOutput());
    Logger.recordOutput("Intake/Pivot/Posistion", getPivotAngleDeg());
    Logger.recordOutput("Intake/Pivot/Goal", goal);

    Logger.recordOutput("Intake/Rollers/Voltage", getRollersVoltage());
    Logger.recordOutput("Intake/Rollers/Has Coral", hasCoral());
    Logger.recordOutput("Intake/Rollers/CAN Range/Distance", mCANRange.getDistance().getValueAsDouble());
    Logger.recordOutput("Intake/Rollers/CAN Range/Detects Object", mCANRange.getIsDetected().getValue());
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {setPID(nTuneableP.get(), nTuneableD.get());
        }, nTuneableP, nTuneableD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {setFF(nTuneableS.get(), nTuneableV.get(), nTuneableG.get());
        },
        nTuneableS, nTuneableV, nTuneableG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setConstraints(nTuneableVelocity.get(), nTuneableAccel.get());
        }, nTuneableVelocity, nTuneableAccel);

    if (isAtLimit(getPivotVoltage())) stopPivotMotor();

    // if(hasCoral()) {
    //   setSlot(1);
    // }
    // else {setSlot(0);}

    // switch(slot) {
    //   case 0:
    //     setPIDandFF(Pivot.kP0, Pivot.kD0, Pivot.kMV0, Pivot.kMA0, Pivot.kS0, Pivot.kV0, Pivot.kG0);
    //     CGoffset = Pivot.kCGAngleOffset;
    //     break;
    //   case 1:
    //     setPIDandFF(Pivot.kP1, Pivot.kD1, Pivot.kMV1, Pivot.kMA1, Pivot.kS1, Pivot.kV1, Pivot.kG1);
    //     CGoffset = Pivot.kCGCoralAngleOffset;
    //     break;
    //   default:
    //     Logger.recordOutput("STOP DUMBAHH", "ELEVATOR");
    // }
  }
}
