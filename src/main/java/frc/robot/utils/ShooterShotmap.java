package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShooterShotmap {
    private InterpolatingTreeMap<Double, ShooterSetpoint> map;
    
    public ShooterShotmap(double[][] pDataPoints) {
        map = new InterpolatingTreeMap<>((a, b, t) -> a + (b-a) * t, ShooterSetpoint::interpolate);
        for (double[] iDataPoint : pDataPoints) {
            try {
                if(iDataPoint.length != 3) {
                    throw new IllegalArgumentException("FAILED TO ADD DATAPOINT! Invalid array length in the shotmap! Expecting 3 parameters (distance, rpm, angle) but " + iDataPoint.length + " datapoints were given");
                } else {
                    map.put(Double.valueOf(iDataPoint[0]), new ShooterSetpoint(iDataPoint[1], iDataPoint[2]));
                }
            } catch (IllegalArgumentException e) {
                System.err.println(e.getLocalizedMessage());
            }
        }
    }

    public ShooterSetpoint getSetpoints(double pDistance) {
        return map.get(Double.valueOf(pDistance));
    }

    public double getAngle(double pDistance) {
        return getSetpoints(pDistance).mAngle;
    }

    public double getRPM(double pDistance) {
        return getSetpoints(pDistance).mRPM;
    }
}
