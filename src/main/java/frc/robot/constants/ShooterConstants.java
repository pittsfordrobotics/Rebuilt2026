package frc.robot.constants;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;

public class ShooterConstants {
    public static final int UPTAKE_MOTOR = 31;
    public static final int[] SHOOTER_MOTORS = {32, 33, 34};
    public static final int HOOD_ACTUATOR_L = 0;
    public static final int HOOD_ACTUATOR_R = 1;

    public static final int STALL_LIMIT = 40;
    public static final int FREE_LIMIT = 40;

    public static final AngularVelocity kFreeSpeed = RPM.of(6000);

    public static final double UPTAKE_SPEED = .25;
    public static final double SHOOTER_SPEED = .25;
    public static final double IS_AT_SPEED_PERCENTAGE = .8;
}
