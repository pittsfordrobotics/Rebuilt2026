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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.lib.util.ShooterHelpers;

public class Hood extends SubsystemBase {

  @Logged(name="Hood Actuator")
	final Servo hood_L = new Servo(ShooterConstants.HOOD_ACTUATOR_L);
	final Servo hood_R = new Servo(ShooterConstants.HOOD_ACTUATOR_R);
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
		return run(() -> {hood_L.set(position.getAsDouble()); hood_R.set(position.getAsDouble());});
	}

  public Command runHoodForShoot(Supplier<Pose2d> currentPose){
    return runHood(() -> getHoodHeight(currentPose));
  }

  private double getHoodHeight(Supplier<Pose2d> currentPose){
    double hubDist = ShooterHelpers.getHubDistInches(currentPose);
    // System.out.println("\n\n\n\n IN HOOD FOR SHOOT hub dist:" + hubDist);
    if (hubDist < 100){
      return 0.2;
    }
    return 0.35;
  }
}
