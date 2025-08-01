// REBELLION 10014

package frc.robot.systems.shooter.flywheel;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FlywheelConstants {

  public static int kSensorPort = 0;
  public static int kTopMotorPort = 32;
  public static int kBottomMotorPort = 33;

  public static final IndexerHardware kIndexerHardware =
      new IndexerHardware(false, 2, IdleMode.kCoast, 60, 2);

  public record IndexerHardware(
      boolean invert,
      double gearing,
      IdleMode idleMode,
      int smartCurrentLimitAmps,
      int secondaryCurrentLimitAmps) {}
  ;

  public record SensorHardware(double hysteresis, double threshold, double minSignalStrength) {}
  ;
}
