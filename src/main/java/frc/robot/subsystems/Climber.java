// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR);
    private GenericEntry climberSpeed;

    /** Creates a new Climber. */
    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true));
        
        climberMotor.getConfigurator().apply(config);

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
