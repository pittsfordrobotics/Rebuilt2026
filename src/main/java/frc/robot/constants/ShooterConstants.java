package frc.robot.constants;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;

public class ShooterConstants {
    public static final int UPTAKE_MOTOR = 31;
    // public static final int[] SHOOTER_MOTORS = {32, 33, 34}; //TODO: put back when other two motors are in place
    public static final int[] SHOOTER_MOTORS = {33};

    public static final int STALL_LIMIT = 40;
    public static final int FREE_LIMIT = 40;

    public static final AngularVelocity kFreeSpeed = RPM.of(6000);
}
