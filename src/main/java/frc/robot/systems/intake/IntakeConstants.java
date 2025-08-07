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
      kRangeConfig.ProximityParams.ProximityThreshold = 0.06;
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
    public static final double kMotorEncoderOffsetRev = 0.4887793;
    public static final double kLowerLimitDeg = -30;
    public static final double kUpperLimitDeg = 80;
    public static final double kCGAngleOffset = 27.0;
    public static final double kCGCoralAngleOffset = 23.0;
    public static final double kTolerance = 5.0;

    public static final double kP0 = 0.06;
    public static final double kD0 = 0.0;
    public static final double kMV0 = 300.0;
    public static final double kMA0 = 600.0;

    public static final double kS0 = 0.0;
    public static final double kG0 = 0.9;
    public static final double kV0 = 0.0;
    public static final double kA0 = 0.0;

    public static final double kP1 = 0.05;
    public static final double kD1 = 0.0;
    public static final double kMV1 = 400.0;
    public static final double kMA1 = 600.0;

    public static final double kS1 = 0.0;
    public static final double kG1 = 1.0;
    public static final double kV1 = 0.0;
    public static final double kA1 = 0.0;

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();
    public static final double kPositionConversionFactor = 360;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public enum Setpoints {
      IntakeAlgae(26),
      IntakeCoral(-30),
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
