package frc.robot.systems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static int kMotorPort = 0;

  public static Rotation2d kMaxPosistion = Rotation2d.fromRotations(0.0);
  public static Rotation2d kMinPosistion = Rotation2d.fromRotations(0.0);

  public static ArmHardware armhardware = new ArmHardware(false, 1 / 25.0, IdleMode.kCoast, 0, 0);

  public static ArmControllerConfig controllerConfig =
      new ArmControllerConfig(0, 0, 0, 0, 0, 0, 0, 0);

  public static EncoderHardware encoderHardware =
      new EncoderHardware(Rotation2d.fromRotations(0.0));

  public record ArmHardware(
      boolean invert,
      double gearing,
      IdleMode idleMode,
      int smartCurrentLimitAmps,
      int secondaryCurrentLimitAmps) {}
  ;

  public record ArmControllerConfig(
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
