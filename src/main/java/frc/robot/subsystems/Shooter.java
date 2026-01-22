// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */


final TalonFX shooterMotor = new TalonFX(0);

  public Shooter() {
    // instantiate motor controllers
      shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
  }


 
  public Command runShooter(DoubleSupplier speed){
    return run(() -> shooterMotor.set(speed.getAsDouble())).finallyDo(() -> shooterMotor.set(0));
  }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
