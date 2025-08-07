package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;
    public static final double kScoringOffsetMeters = -0.3 + Units.inchesToMeters(2.0);

    public static final double kBargeXPosition = Constants.kFieldLengthMeters / 2.0;
    public static final double kBargeLength = Units.inchesToMeters(146.5);

    public static final double kBlueBargeEdge = Constants.kFieldWidthMeters - kBargeLength;
    public static final double kRedBargeEdge = 0.0 + kBargeLength;

    public static final double algaeDiameter = Units.inchesToMeters(15);
}