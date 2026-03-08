// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;
import frc.robot.lib.util.ShooterHelpers;

public class Hood extends SubsystemBase {

  @Logged(name="Left Actuator")
	final Servo hood_L = new Servo(HoodConstants.HOOD_ACTUATOR_L);

	@Logged(name="Right Actuator")
  final Servo hood_R = new Servo(HoodConstants.HOOD_ACTUATOR_R);

  @Logged(name="Is at setpoint")
  public boolean isAtSetpoint() {
    // Unfortunately the actuators only report what the setpoint is, not what position currently at.
    // Attempt to calculate if the actuators are at their setpoint by approximating
    // how long it takes to change positions.
    double distanceToTravel = Math.abs(previousSetPoint - currentSetPoint);
    double timeNeeded = distanceToTravel / HoodConstants.POSITION_CHANGE_RATE;
    double currentTime = Timer.getFPGATimestamp();
    
    return currentTime > (lastSetTimeSeconds + timeNeeded);
  }

  private double previousSetPoint = 0;
  private double currentSetPoint = 0.2; // When starting, assume we're fully lowered.
  private double lastSetTimeSeconds = 0;

  private GenericEntry hoodPercent;
  /** Creates a new Hood. */
  public Hood() {
    hoodPercent = Shuffleboard.getTab("testing").add("Hood Pos Percentage", 0.5).getEntry(); // 0.2 to 0.4
		Shuffleboard.getTab("testing").add("Set Hood Pos", this.runHood(() -> hoodPercent.getDouble(0.5)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runHood(DoubleSupplier position) {
		return run(() -> {setHoodPosition(position.getAsDouble());});
	}

  public Command runHoodForShoot(Supplier<Pose2d> currentPose) {
    return runHood(() -> calculateHoodPositionFromPose(currentPose));
  }

  private void setHoodPosition(double position) {
    hood_L.set(position);
    hood_R.set(position);
    if (position != currentSetPoint) {
      previousSetPoint = currentSetPoint;
      currentSetPoint = position;
      lastSetTimeSeconds = Timer.getFPGATimestamp();
    }
  }

  private double calculateHoodPositionFromPose(Supplier<Pose2d> currentPose) {
    double hubDistInches = ShooterHelpers.getHubDistInches(currentPose);
    if (hubDistInches < HoodConstants.HUB_DISTANCE_FOR_NEAR_SHOOTING_INCHES) {
      return HoodConstants.CLOSE_SHOOTING_SETPOINT;
    }

    return HoodConstants.FAR_SHOOTING_SETPOINT;
  }
}
