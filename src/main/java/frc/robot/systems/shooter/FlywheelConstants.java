package frc.robot.systems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class FlywheelConstants {

  public class topFlywheel {
    public static final int kMotorID = 32;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final boolean kMotorInverted = false;
    public static final int kCurrentLimit = 60;

    public static final int kMaxRPM = 6204; // Measured empirically with 2 second accel timer, increase if shots are too weak 

    public static final double kToleranceRPM = 50;
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SparkFlexConfig kMotorConfig = new SparkFlexConfig();

    public enum Voltage {
      IntakeAlgae(6),
      scoreProcessor(-6),
      BasicShootAlgae(-12);

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

  public class bottomFlywheel {
    public static final int kMotorID = 33;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final boolean kMotorInverted = true;
    public static final int kCurrentLimit = 60;

    public static final int kMaxRPM = 6284; // Measured empirically with 2 second accel timer, increase if shots are too weak 

    public static final double kToleranceRPM = 50;
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SparkFlexConfig kMotorConfig = new SparkFlexConfig();

    public enum Voltage {
      IntakeAlgae(6),
      scoreProcessor(-6),
      BasicShootAlgae(-12);

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

  public class indexer {
    public static final int kMotorID = 34;
    public static final int kSensorPort = 1;

    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final boolean kMotorInverted = false;
    public static final int kCurrentLimit = 60;

    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SparkFlexConfig kMotorConfig = new SparkFlexConfig();

    public enum Voltage {
      IndexAlgae(4),
      HoldAlgae(1),
      scoreProcessor(-6),
      FireAlgae(-12);

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
