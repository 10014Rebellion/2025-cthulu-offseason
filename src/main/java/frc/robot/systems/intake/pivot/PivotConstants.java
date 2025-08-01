package frc.robot.systems.intake.pivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
  public static int kMotorPort = 41;

  public static Rotation2d kMaxPosistion = Rotation2d.fromDegrees(70.0);
  public static Rotation2d kMinPosistion = Rotation2d.fromDegrees(-30.0);

  public static PivotHardware pivotHardware =
      new PivotHardware(false, 1 / 25.0, IdleMode.kCoast, 40, 0);

  public static PivotControllerConfig controllerConfig =
      new PivotControllerConfig(0, 0, 0, 0, 0, 1.32, 3.06, 0);
  // Calculated kG: 1.32, kV: 3.06, kA: 0.37

  public static EncoderHardware encoderHardware =
      new EncoderHardware(Rotation2d.fromRotations(0.1413399));

  public record PivotHardware(
      boolean invert,
      double gearing,
      IdleMode idleMode,
      int smartCurrentLimitAmps,
      int secondaryCurrentLimitAmps) {}
  ;

  public record PivotControllerConfig(
      double kP,
      double kI,
      double kD,
      double kMaxVelo,
      double kMaxAccel,
      double kG,
      double kV,
      double kS) {}
  ;

  public record EncoderHardware(Rotation2d offset) {}
  ;
}
