package frc.robot.constants;

public class HoodConstants {
    public static final int HOOD_ACTUATOR_L = 0;
    public static final int HOOD_ACTUATOR_R = 1;

    // NOTE:
    // The near vs far shooting distance needs to be tuned.
    // Minimum hub distance should be closer to about 60.
    // For now (RIT competition), we're forcing the hood to always
    // remain at 0.35. This will need to be changed later!
    public static final double HUB_DISTANCE_FOR_NEAR_SHOOTING_INCHES = 100; // Should be moved closer - 60?
    public static final double CLOSE_SHOOTING_SETPOINT = 0.35; // Should be changed back to 0.2
    public static final double FAR_SHOOTING_SETPOINT = 0.35; // Works well in general except very close to hub.
    public static final double PASSING_SETPOINT = 0.35;

    // This rate is the approximate speed the actuator can
    // change position. It is in units per second.
    // For example, if it takes 1 second to go from 0.2 to 0.35,
    // the rate would be 0.15.
    public static final double POSITION_CHANGE_RATE = 0.15;
}