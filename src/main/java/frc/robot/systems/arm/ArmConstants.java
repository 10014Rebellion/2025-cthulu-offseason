package frc.robot.systems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static int kMotorID = 31;

  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static boolean kMotorInverted = false;
  public static int kCurrentLimit = 60;

  public static double kP = 0.8;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double kMaxAcceleration = 2000;
  public static double kMaxVelocity = 700;

  public static double kTolerance = 1.0;

  public static double kUpperLimitDeg = 80; // Mechanical Limit: 90
  public static double kLowerLimitDeg = -37; // Mechanical Limit: -40

  public static double kEncoderOffsetRotations = 0.8213854;
  public static boolean kEncoderInverted = true;

  public static double kPositionConversionFactor = 360.0;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

  public static double kS = 3.0;
  public static double kG = 0.5;
  public static double kV = 40.0;
  public static double kA = 0.0;
  // Calculated kG: 0.24, kV: 39.18, kA: 0.06

  public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

  public enum Setpoints {
    Hold(Rotation2d.fromDegrees(45)),
    Intake(Rotation2d.fromDegrees(-40)),
    L2Algae(Rotation2d.fromDegrees(90)),
    L3Algae(Rotation2d.fromDegrees(90));

    public final Rotation2d setpoint;

    private Setpoints(Rotation2d setpoint) {
      this.setpoint = setpoint;
    }

    public Rotation2d getPos() {
      return this.setpoint;
    }
  };

  static {
    kMotorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit).inverted(kMotorInverted);

    kMotorConfig
        .absoluteEncoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor)
        .zeroOffset(kEncoderOffsetRotations);
  }
}
