// REBELLION 10014

package frc.robot.systems.intake.rollers;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class RollersConstant {

  public static int kSensorPort = 42;
  public static int kMotorPort = 43;

  public static RollerHardware rollerHardware = new RollerHardware(false, 0, IdleMode.kCoast, 0, 0);

  public static SensorHardware sensorHardware = new SensorHardware(0, 0, 0);

  public record RollerHardware(
      boolean invert,
      double gearing,
      IdleMode idleMode,
      int smartCurrentLimitAmps,
      int secondaryCurrentLimitAmps) {}
  ;

  public record SensorHardware(double hysteresis, double threshold, double minSignalStrength) {}
  ;
}
