// REBELLION 10014

package frc.robot.systems.shooter.indexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerConstants {

  public static int kSensorPort = 0;
  public static int kLeftMotorPort = 32;
  public static int kRightMotorPort = 33;

  public static final IndexerHardware kIndexerHardware = new IndexerHardware(false, 2, IdleMode.kCoast, 60, 2);

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
