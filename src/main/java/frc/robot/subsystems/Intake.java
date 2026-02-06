// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIDConstants;


public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor = new SparkFlex(CanIDConstants.IntakeCanID, MotorType.kBrushless);
  // private SparkFlex indexMotor = new SparkFlex(12, MotorType.kBrushless);




  /** Creates a new intake. */
  public Intake() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(20);
    config.idleMode(IdleMode.kBrake);

    REVLibError err = intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("Init error code: " + err.value);

    // REVLibError err2 = indexMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // System.out.println("Init error code: " + err2.value);
  }


  public Command runIntake(DoubleSupplier speed){
    return run(() -> intakeMotor.set(speed.getAsDouble())).finallyDo(() -> intakeMotor.set(0));
  }
  

  // public Command runIndex(DoubleSupplier speed){
  //   return run(() -> indexMotor.set(speed.getAsDouble())).finallyDo(() -> indexMotor.set(0));
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
