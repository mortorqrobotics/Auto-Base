package org.team1515.botmitzvah.Utils;

public class Utilities {

    /**
     * @param value    input value
     * @param deadband amount of deadband
     * @return double value accounted for base error
     */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /*
     * Returns true if error is within epsilon
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}