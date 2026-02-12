// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private SparkFlex climberMotor = new SparkFlex(0, null);
  private GenericEntry climberSpeed;
  /** Creates a new Climber. */
  public Climber() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(20);
    config.idleMode(IdleMode.kBrake);

    REVLibError err3 = climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("Init error code: " + err3.value);

        climberSpeed = Shuffleboard.getTab("testing").add("Climber Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Climber", this.runClimber(() -> climberSpeed.getDouble(0.25)));

  }

  public Command runClimber(DoubleSupplier speed) {
    return run(() -> climberMotor.set(speed.getAsDouble())).finallyDo(() -> climberMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
