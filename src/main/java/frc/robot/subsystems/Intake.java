// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private TalonFX driveMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE);
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT);
    private GenericEntry intakeSpeed;



    /** Creates a new intake. */
    public Intake() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        driveMotor.getConfigurator().apply(driveConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);

        intakeSpeed = Shuffleboard.getTab("testing").add("Intake Motor Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Intake", this.runIntake(() -> intakeSpeed.getDouble(0.25)));
    }


    public Command runIntake(DoubleSupplier speed) {
        return run(() -> driveMotor.set(speed.getAsDouble())).finallyDo(() -> driveMotor.set(0));
    }

    public Command pivotOut() {
        return run(() -> {
            PositionVoltage control = new PositionVoltage(1).withSlot(0);
            pivotMotor.setControl(control);
        });
    }

    public Command pivotIn() {
        return run(() -> {
            PositionVoltage control = new PositionVoltage(0).withSlot(0);
            pivotMotor.setControl(control);
        });
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
