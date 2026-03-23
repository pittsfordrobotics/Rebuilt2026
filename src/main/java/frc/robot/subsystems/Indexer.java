// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.lib.util.TalonConfigurator;


public class Indexer extends SubsystemBase {
    @Logged(name = "Index Motor")
    private TalonFX indexMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR);
    private GenericEntry indexSpeed;

    @Logged(name = "Current Command")
    public String IndexerCurrentCommand() {
        if (this.getCurrentCommand() == null) {
            return "none";
        }
        return this.getCurrentCommand().getName();
    }

    /** Creates a new intake. */
    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));
        
        indexMotor.getConfigurator().apply(config);
        TalonConfigurator.reduceCommonStatusFrameFrequencies(indexMotor);

        indexSpeed = Shuffleboard.getTab("testing").add("Index Speed", .9).getEntry();

        Shuffleboard.getTab("testing").add("Run Indexer", this.runIndex());
    }

    public Command runIndex(DoubleSupplier speed) {
        return run(() -> indexMotor.set(speed.getAsDouble())).finallyDo(() -> indexMotor.set(0));
    }

    public Command runIndex() {
        return this.runIndex(() -> indexSpeed.getDouble(0.9));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
