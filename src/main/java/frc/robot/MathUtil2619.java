package frc.robot;

public class MathUtil2619 {
    /**
     * Gets shortest distance to an angle (copied from SteamBot)
     * 
     * @param current angle (degrees)
     * @param desired angle (degrees)
     * @return +1 for CW, -1 for CCW
     */
    public static double calcDirection(double current, double desired) {
        current = Math.toRadians(current);
        desired = Math.toRadians(desired);
        double current_x = Math.cos(current);
        double current_y = Math.sin(current);
        double desired_x = Math.cos(desired);
        double desired_y = Math.sin(desired);
        double direction = Math.signum(current_x * desired_y - desired_x * current_y);
        return direction;
    }

    /**
     * Calls {@link #delin(double, double, double, double)} with preset parameters
     * 
     * @param speed the speed to adjust
     * @return the adjusted speed
     */
    public static double adjSpeed(double speed) {
        double dB = 0.1;
        double root = 1;
        double power = 3;
        speed = MathUtil2619.deadband(speed, dB);
        speed = MathUtil2619.delin(speed, dB, root, power);
        return speed;
    }

    /**
     * Removes joystick (generally use adjust speed though)
     * 
     * @param speed number within the range [-1, 1]
     * @param dead  width of the deadband, i.e. it will be the range [-deadband,
     *              deadband]
     * @return adjusted speed
     * @see {@link #adjSpeed(double)}, which this is used in
     */
    public static double deadband(double speed, double dead) {
        return (Math.abs(speed) < dead) ? 0 : speed;
    }

    /**
     * Applies delinearization
     * 
     * @param speed the speed to adjust
     * @param dead  width of the deadband for {@link #deadband(double, double)}
     * @param root
     * @param pwr
     * @return Value adjusted with delin and deadband
     * @see {@link #adjSpeed(double)}
     */
    public static double delin(double speed, double dead, double root, double pwr) {
        double evn = (pwr / root) % 2; // 1
        double invdB = Math.pow(1 - dead, -1); // 1/0.9
        double cons = pwr / root; // 3
        if (speed != 0) { // Makes sure deadband doesn't bypass the calculations
            if (speed > 0) // Speed is greater than zero and so there are no exceptions
                return Math.pow(invdB * (speed - dead), cons);
            else if (evn != 0) // Less than zero, checks for even power
                return Math.pow(invdB * (speed + dead), cons);
            else // To stay negative, a "-" must be put at the beginning to maintain negativity
                 // of speed
                return -Math.pow(invdB * (speed + dead), cons);
        } else
            return 0;
    }

    /**
     * Rescales a value on the range [min, max] to be on the range [0, 1]
     * 
     * @param input
     * @param max
     * @param min
     * @return value on range [0, 1]
     */
    public static double lerp(double input, double max, double min) {
        return (input - min) / (max - min);
    }
}
