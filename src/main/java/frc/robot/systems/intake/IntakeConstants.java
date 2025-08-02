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
    public static final boolean kEncoderInverted = false;
    public static final double kMotorEncoderOffsetRev = 0.1413399;
    public static final double kLowerLimitDeg = -30;
    public static final double kUpperLimitDeg = 70;

    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    static {
      kMotorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
      kMotorConfig.absoluteEncoder.inverted(kEncoderInverted).zeroOffset(kMotorEncoderOffsetRev);
    }
  }

  public class Roller {
    public static final int kMotorID = 43;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final boolean kMotorInverted = false;

    public static final int kCurrentLimit = 50;

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    static {
      kMotorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);
    }
  }
}
