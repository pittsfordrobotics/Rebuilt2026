// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.lib.util.TalonConfigurator;


public class Intake extends SubsystemBase {
    @Logged(name = "Intake Drive")
    private TalonFX driveMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE);
    @Logged(name = "Pivot Motor")
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT);
    private GenericEntry intakeSpeed;

    @Logged(name = "Current Command")
    public String IntakeCurrentCommand() {
        if (this.getCurrentCommand() == null) {
            return "none";
        }
        return this.getCurrentCommand().getName();
    }

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
                .withKP(.5)
                .withKI(0)
                .withKD(0)
            ).withSlot1(
                new Slot1Configs()
                .withKP(.01)
                .withKI(0)
                .withKD(0)
            ).withSlot2(
                new Slot2Configs()
                .withKP(1.5)
                .withKI(0)
                .withKD(0)
            );

        driveMotor.getConfigurator().apply(driveConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);

        TalonConfigurator.reduceCommonStatusFrameFrequencies(driveMotor);
        TalonConfigurator.reduceCommonStatusFrameFrequencies(pivotMotor);

        intakeSpeed = Shuffleboard.getTab("testing").add("Intake Motor Speed", 1).getEntry();
        Shuffleboard.getTab("testing").add("Run Intake", this.runIntake(() -> intakeSpeed.getDouble(0.9)));

        // pivotOutSpeed = Shuffleboard.getTab("testing").add("Intake Pivot Out Speed", .4).getEntry();
        // pivotInSpeed = Shuffleboard.getTab("testing").add("Intake Pivot In Speed", .2).getEntry();
        Shuffleboard.getTab("testing").add("Pivot Out", this.pivotOut());
        Shuffleboard.getTab("testing").add("Pivot In", this.pivotIn());
    }


    public Command runIntake(DoubleSupplier speed) {
        return run(() -> driveMotor.set(speed.getAsDouble())).finallyDo(() -> driveMotor.set(0));
    }

    public Command runIntake() {
        return runIntake(() -> intakeSpeed.getDouble(1));
    }

    public Command pivotOut() {
        return runOnce(() -> {
            PositionVoltage control = new PositionVoltage(IntakeConstants.PIVOT_EXTENDED).withSlot(0);
            pivotMotor.setControl(control);
        });
    }

    public Command pivotIn() {
        return runOnce(() -> {
            PositionVoltage control = new PositionVoltage(0).withSlot(0);
            pivotMotor.setControl(control);
        });
    }

    public Command agitate() {
        //I'M AGITATED
        // return runOnce(() -> pivotMotor.set(-.05)).andThen(
        //     Commands.waitSeconds(.5),
        //     runOnce(() -> pivotMotor.set(.05)),
        //     Commands.waitSeconds(.5)).repeatedly()
        //     .finallyDo(() -> pivotMotor.set(0));
        return runOnce(() -> {
                pivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_AGITATE2).withSlot(2));
                driveMotor.set(0.9);
            }).andThen(Commands.waitSeconds(.2),
            runOnce(() -> pivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_AGITATE1).withSlot(2))),
            Commands.waitSeconds(.2)).repeatedly()
            .finallyDo(() -> {
                pivotOut();
                driveMotor.set(0);}
        );
    }

    public Command lemonSqueeze() {
        return runOnce(() -> {
            pivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_HOME).withSlot(1));
        });
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Logged(name = "Intake drive")
    public TalonFX getIntakeMotor(){
        return driveMotor;
    }

    @Logged(name = "Intake pivot")
    public TalonFX getPivotMotor(){
        return pivotMotor;
    }

    public Command resetEncoder() {
        return runOnce(() -> {
            if(DriverStation.isDisabled()) {
                this.pivotMotor.getConfigurator().setPosition(0);
            }
        });
    }
}
