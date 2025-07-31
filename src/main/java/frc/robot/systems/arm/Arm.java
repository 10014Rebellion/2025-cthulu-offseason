package frc.robot.systems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmConstants.EncoderHardware;
import frc.robot.systems.arm.ArmConstants.ArmHardware;
import frc.robot.utils.LoggedTunableNumber;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;


import static frc.robot.systems.arm.ArmConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Arm extends SubsystemBase{

    public enum ArmGoal {
        INTAKE(Rotation2d.fromDegrees(0.0)),
        SHOOT(Rotation2d.fromDegrees(0.0));

        private double goalRotations;

        private ArmGoal(Rotation2d goalRotations) {
            this.goalRotations = goalRotations.getRotations();
        }

        public double getGoalRotations() {
            return goalRotations;
        }
    }

    private final SparkMax kMotor = new SparkMax(kMotorPort, MotorType.kBrushless);
    
    private final ArmHardware kMotorHardware;
    private final EncoderHardware kEncoderHardware;

    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    private ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, null);
    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

    private LoggedTunableNumber kTuneableP = new LoggedTunableNumber("Arm/Tuneables/kP", 0.0);
    private LoggedTunableNumber kTuneableI = new LoggedTunableNumber("Arm/Tuneables/kI", 0.0);
    private LoggedTunableNumber kTuneableD = new LoggedTunableNumber("Arm/Tuneables/kD", 0.0);
    private LoggedTunableNumber kTuneableVelocity = new LoggedTunableNumber("Arm/Tuneables/kVelocity", 0.0);
    private LoggedTunableNumber kTuneableAccel = new LoggedTunableNumber("Arm/Tuneables/kAccel", 0.0);
    private LoggedTunableNumber kTuneableS = new LoggedTunableNumber("Arm/Tuneables/kS", 0.0);
    private LoggedTunableNumber kTuneableV = new LoggedTunableNumber("Arm/Tuneables/kV", 0.0);
    private LoggedTunableNumber kTuneableG = new LoggedTunableNumber("Arm/Tuneables/kG", 0.0);

    @AutoLogOutput(key = "Arm/Goal")
    private ArmGoal goal = null;
 
    public Arm(){
        this.kMotorHardware = ArmConstants.armhardware;
        this.kEncoderHardware = ArmConstants.encoderHardware;

        motorConfig.inverted(kMotorHardware.invert());
        motorConfig.smartCurrentLimit(kMotorHardware.smartCurrentLimitAmps());
        motorConfig.secondaryCurrentLimit(kMotorHardware.secondaryCurrentLimitAmps());
        motorConfig.idleMode(kMotorHardware.idleMode());

        kMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        controller.setPID(controllerConfig.kP(), controllerConfig.kI(), controllerConfig.kD());
        controller.setConstraints(new Constraints(controllerConfig.kMaxVelo(), controllerConfig.kMaxAccel()));
        feedforward.setKs(controllerConfig.kS());
        feedforward.setKg(controllerConfig.kG());
        feedforward.setKv(controllerConfig.kV());
    }

    public Rotation2d getPosistion(){
        return Rotation2d.fromRotations(kMotor.getAbsoluteEncoder().getPosition()).minus(encoderHardware.offset());
    }

    public double getVelocity(){
        return kMotor.getAbsoluteEncoder().getVelocity();
    }

    public double getAppliedVoltage() {
        return kMotor.getAppliedOutput() * kMotor.getBusVoltage();
    }

    public double getOutputAmps() {
        return kMotor.getOutputCurrent();
    }

    public double getTemp() {
        return kMotor.getMotorTemperature();
    }

    public void setVoltage(double volts){
        volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
        kMotor.setVoltage(volts);
    }

    public void setGoal(ArmGoal desiredGoal) {
        goal = desiredGoal;
    }

    public Command setPosistion(double goal){
        return new FunctionalCommand(
            () -> {
                controller.reset(getPosistion().getRotations());
                controller.setGoal(goal);
                controller.setTolerance(2 / 360.0);
            }, 
            () -> {
                double demand = controller.calculate(getPosistion().getRotations(), goal);
                setVoltage(demand);
            }, 
            (interrupted) -> {
                
            }, 
            () -> controller.atGoal(), 
            this)
            
            .andThen(
                
                new FunctionalCommand(
                    () -> {

                    }, 
                    () -> {
                        double demand = feedforward.calculate(getPosistion().getRadians(), getVelocity());
                        setVoltage(demand);
                    }, 
                    (interrupted) -> {
                        
                    }, 
                    () -> false, 
                this)
            );
    }

    public void setPID(double kP, double kI, double kD){
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setConstraints(double kMaxVelo, double kMaxAccel){
        controller.setConstraints(new Constraints(kMaxVelo, kMaxAccel));
    }

    public void setFF(double kS, double kV, double kG){
        feedforward.setKg(kG);
        feedforward.setKv(kV);
        feedforward.setKs(kS);
    }


    @Override
    public void periodic(){

        Logger.recordOutput("Arm/Temperature C*", getTemp());
        Logger.recordOutput("Arm/Velocity Rot.s", getVelocity());
        Logger.recordOutput("Arm/Voltage", getAppliedVoltage());
        Logger.recordOutput("Arm/OutputAmps", getOutputAmps());
        Logger.recordOutput("Arm/Posistion", getPosistion().getDegrees());

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                setPID(kTuneableP.get(), kTuneableI.get(), kTuneableD.get());
            }, kTuneableP, kTuneableI, kTuneableD);


        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                setFF(kTuneableS.get(), kTuneableV.get(), kTuneableG.get());
            }, kTuneableS, kTuneableV, kTuneableG);


        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                setConstraints(kTuneableVelocity.get(), kTuneableAccel.get());
            }, kTuneableVelocity, kTuneableAccel);

        if (goal != null){
            setPosistion(goal.getGoalRotations());
        }
    }
}
