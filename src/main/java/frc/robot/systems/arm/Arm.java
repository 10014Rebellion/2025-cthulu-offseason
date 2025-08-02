package frc.robot.systems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final SparkMax mArmMotor;
  private final AbsoluteEncoder mArmEncoder;

  private LoggedTunableNumber nTuneableP =
      new LoggedTunableNumber("Arm/Tuneables/kP", ArmConstants.kP);
  private LoggedTunableNumber nTuneableI =
      new LoggedTunableNumber("Arm/Tuneables/kI", ArmConstants.kI);
  private LoggedTunableNumber nTuneableD =
      new LoggedTunableNumber("Arm/Tuneables/kD", ArmConstants.kD);
  private LoggedTunableNumber nTuneableVelocity =
      new LoggedTunableNumber("Arm/Tuneables/kVelocity", ArmConstants.kV);
  private LoggedTunableNumber nTuneableAccel =
      new LoggedTunableNumber("Arm/Tuneables/kAccel", ArmConstants.kA);
  private LoggedTunableNumber nTuneableS =
      new LoggedTunableNumber("Arm/Tuneables/kS", ArmConstants.kS);
  private LoggedTunableNumber nTuneableV =
      new LoggedTunableNumber("Arm/Tuneables/kV", ArmConstants.kV);
  private LoggedTunableNumber nTuneableG =
      new LoggedTunableNumber("Arm/Tuneables/kG", ArmConstants.kG);
  private LoggedTunableNumber nTuneableSetpoint =
      new LoggedTunableNumber("Arm/Tuneables/Setpoint", 0.0);

  private ProfiledPIDController controller =
      new ProfiledPIDController(
          nTuneableP.get(),
          nTuneableI.get(),
          nTuneableD.get(),
          new TrapezoidProfile.Constraints(nTuneableVelocity.get(), nTuneableAccel.get()));
  private ArmFeedforward feedforward =
      new ArmFeedforward(nTuneableS.get(), nTuneableG.get(), nTuneableV.get());

  public Arm() {
    this.mArmMotor = new SparkMax(ArmConstants.kMotorID, ArmConstants.kMotorType);
    this.mArmEncoder = mArmMotor.getAbsoluteEncoder();
    this.mArmMotor.configure(
        ArmConstants.kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller.setPID(nTuneableP.get(), nTuneableI.get(), nTuneableD.get());
    controller.setConstraints(new Constraints(nTuneableVelocity.get(), nTuneableAccel.get()));
    feedforward.setKs(nTuneableS.get());
    feedforward.setKg(nTuneableG.get());
    feedforward.setKv(nTuneableV.get());
  }

  public Rotation2d getPosistion() {
    return Rotation2d.fromDegrees(mArmMotor.getAbsoluteEncoder().getPosition());
  }

  public double getPosistionDegrees() {
    return mArmMotor.getAbsoluteEncoder().getPosition();
  }

  public double getPosistionRadians() {
    return Units.degreesToRadians(mArmMotor.getAbsoluteEncoder().getPosition());
  }

  public double getVelocity() {
    return mArmMotor.getAbsoluteEncoder().getVelocity();
  }

  public double getAppliedVoltage() {
    return mArmMotor.getAppliedOutput() * mArmMotor.getBusVoltage();
  }

  public double getOutputAmps() {
    return mArmMotor.getOutputCurrent();
  }

  public double getTemp() {
    return mArmMotor.getMotorTemperature();
  }

  public void setVoltage(double volts) {
    volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
    mArmMotor.setVoltage(volts);
  }

  public FunctionalCommand enableFFCmd() {

    return new FunctionalCommand(
        () -> {
          System.out.println("Arm FF Running");
        },
        () -> {
          double calculatedOutput =
              feedforward.calculate(Units.degreesToRadians(getPosistionDegrees()), 0);
          setVoltage(calculatedOutput);
        },
        (interrupted) ->
            setVoltage(feedforward.calculate(Units.degreesToRadians(getPosistionDegrees()), 0)),
        () -> false,
        this);
  }

  public boolean isPIDAtGoal() {
    return controller.atGoal();
  }

  public FunctionalCommand setPIDCmd(double pSetpoint) {
    return new FunctionalCommand(
        () -> {
          controller.reset(getPosistionDegrees());
          controller.setGoal(pSetpoint);
        },
        () -> {
          double encoderReading = getPosistionDegrees();
          double calculatedPID = controller.calculate(encoderReading);
          double calculatedFF =
              feedforward.calculate(
                  Units.degreesToRadians(controller.getSetpoint().position),
                  Units.degreesToRadians(controller.getSetpoint().velocity));

          setVoltage(calculatedPID + calculatedFF);
          // SmartDashboard.putNumber("Wrist/Full Output", calculatedPID + calculatedFF);
          // SmartDashboard.putNumber("Wrist/PID Output", calculatedPID);
          // SmartDashboard.putNumber("Wrist/FF Output", calculatedFF);
        },
        (interrupted) -> setVoltage(0),
        () -> isPIDAtGoal(),
        this);
  }

  public FunctionalCommand setTuneablePIDCmd() {
    return new FunctionalCommand(
        () -> {
          controller.reset(getPosistionDegrees());
          controller.setGoal(nTuneableSetpoint.get());
        },
        () -> {
          double encoderReading = getPosistionDegrees();
          double calculatedPID = controller.calculate(encoderReading);
          double calculatedFF =
              feedforward.calculate(
                  Units.degreesToRadians(controller.getSetpoint().position),
                  Units.degreesToRadians(controller.getSetpoint().velocity));

          setVoltage(calculatedPID + calculatedFF);
          // SmartDashboard.putNumber("Wrist/Full Output", calculatedPID + calculatedFF);
          // SmartDashboard.putNumber("Wrist/PID Output", calculatedPID);
          // SmartDashboard.putNumber("Wrist/FF Output", calculatedFF);
        },
        (interrupted) -> setVoltage(0),
        () -> isPIDAtGoal(),
        this);
  }

  public void setPID(double kP, double kI, double kD) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  public void setConstraints(double kMaxVelo, double kMaxAccel) {
    controller.setConstraints(new Constraints(kMaxVelo, kMaxAccel));
  }

  public void setFF(double kS, double kV, double kG) {
    feedforward.setKg(kG);
    feedforward.setKv(kV);
    feedforward.setKs(kS);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/Temperature C*", getTemp());
    Logger.recordOutput("Arm/Velocity Rot.s", getVelocity());
    Logger.recordOutput("Arm/Voltage", getAppliedVoltage());
    Logger.recordOutput("Arm/OutputAmps", getOutputAmps());
    Logger.recordOutput("Arm/Posistion", getPosistion().getDegrees());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setPID(nTuneableP.get(), nTuneableI.get(), nTuneableD.get());
        },
        nTuneableP,
        nTuneableI,
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
  }
}
