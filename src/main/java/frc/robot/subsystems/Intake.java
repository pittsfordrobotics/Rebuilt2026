// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;


public class Intake extends SubsystemBase {
    private TalonFX driveMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE);
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT);
    private GenericEntry intakeSpeed;
    private GenericEntry pivotInSpeed;
    private GenericEntry pivotOutSpeed;



    /** Creates a new intake. */
    public Intake() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                .withKP(.35)
                .withKI(0)
                .withKD(0)
                .withKV(0) // 12 volts when requesting max RPS
            ).withSlot1(
                new Slot1Configs()
                .withKP(.5)
                .withKI(0)
                .withKD(0)
                .withKV(0)
            );

        driveMotor.getConfigurator().apply(driveConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);

        intakeSpeed = Shuffleboard.getTab("testing").add("Intake Motor Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Intake", this.runIntake(() -> intakeSpeed.getDouble(0.25)));

        // pivotOutSpeed = Shuffleboard.getTab("testing").add("Intake Pivot Out Speed", .4).getEntry();
        // pivotInSpeed = Shuffleboard.getTab("testing").add("Intake Pivot In Speed", .2).getEntry();
        Shuffleboard.getTab("testing").add("Pivot Out", this.pivotOut());
        Shuffleboard.getTab("testing").add("Pivot In", this.pivotIn());
    }


    public Command runIntake(DoubleSupplier speed) {
        return run(() -> driveMotor.set(speed.getAsDouble())).finallyDo(() -> driveMotor.set(0));
    }

    public Command runIntake() {
        return runIntake(() -> intakeSpeed.getDouble(.25));
    }

    public Command pivotOut() {
        return run(() -> {
            PositionVoltage control = new PositionVoltage(15).withSlot(0);
            pivotMotor.setControl(control);
        });
    }

    public Command pivotIn() {
        return run(() -> {
            PositionVoltage control = new PositionVoltage(0).withSlot(1);
            pivotMotor.setControl(control);
        });
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
