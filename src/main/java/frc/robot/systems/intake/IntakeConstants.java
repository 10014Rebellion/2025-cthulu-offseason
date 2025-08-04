package frc.robot.systems.intake;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  public class CANRange {
    public static int kCANRangeID = 42;

    public static final CANrangeConfiguration kRangeConfig = new CANrangeConfiguration();

    static {
      kRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
      kRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
      kRangeConfig.ProximityParams.ProximityThreshold = 0.03;
    }
  }

  public class Pivot {
    public static final int kMotorID = 41;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final boolean kMotorInverted = false;
    public static final int kCurrentLimit = 50;

    public static final double kEncoderConversionFactor = 360;
    public static final boolean kEncoderInverted = true;
    public static final double kMotorEncoderOffsetRev = 0.1413399;
    public static final double kLowerLimitDeg = -25;
    public static final double kUpperLimitDeg = 70;
    public static final double kTolerance = 5.0;

    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();
    public static final double kPositionConversionFactor = 360;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public enum Setpoints {
      IntakeAlgae(15),
      IntakeCoral(-25),
      Processor(30),
      AvoidArm(45),
      ScoreL1(60),
      StowIntake(60);

      public final double setpoint;

      private Setpoints(double setpoint) {
        this.setpoint = setpoint;
      }

      public double getPos() {
        return this.setpoint;
      }
    };

    static {
      kMotorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
      kMotorConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .inverted(kEncoderInverted)
          .zeroOffset(kMotorEncoderOffsetRev);
    }
  }

  public class Roller {
    public static final int kMotorID = 43;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final boolean kMotorInverted = false;

    public static final int kCurrentLimit = 50;

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    public enum Voltage {
      IntakeAlgae(-6),
      IntakeCoral(12),
      HoldCoral(1),
      ScoreProcessor(6),
      ScoreL1(-12);

      public final double voltage;

      private Voltage(double voltage) {
        this.voltage = voltage;
      }

      public double getVoltage() {
        return this.voltage;
      }
    };

    static {
      kMotorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
    }
  }
}
