// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
	/** Creates a new Shooter. */

	private GenericEntry shooterSpeed;
	private GenericEntry uptakeSpeed;

	final TalonFX[] shooterMotors = new TalonFX[ShooterConstants.SHOOTER_MOTORS.length];
	final TalonFX uptakeMotor = new TalonFX(ShooterConstants.UPTAKE_MOTOR);

	public Shooter() {
		TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Coast))
				.withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true));

        TalonFXConfiguration uptakeConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Coast))
				.withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive));

		// instantiate motor controllers
		for (int i = 0; i < shooterMotors.length; i++) {
			shooterMotors[i] = new TalonFX(ShooterConstants.SHOOTER_MOTORS[i]);
			shooterMotors[i].getConfigurator().apply(shooterConfig);
		}

		uptakeMotor.getConfigurator().apply(uptakeConfig);
		Shuffleboard.getTab("testing").add("Run Shooter", this.runShooter());

        shooterSpeed = Shuffleboard.getTab("testing").add("Shooter Motor Speed", .25).getEntry();
		uptakeSpeed = Shuffleboard.getTab("testing").add("Uptake Motor Speed", .25).getEntry();
	}

	public Command runShooter(DoubleSupplier shootSpeed, DoubleSupplier uptakeSpeed) {
		return run(() -> {
			for (TalonFX motor : shooterMotors) {
				motor.set(shootSpeed.getAsDouble());
			}
            uptakeMotor.set(uptakeSpeed.getAsDouble());
		}).finallyDo(() -> {
			for (TalonFX motor : shooterMotors) {
				motor.set(0);
			}
            uptakeMotor.set(0);
		});
	}

    public Command runShooter() {
        return runShooter(() -> shooterSpeed.getDouble(0.25), () -> uptakeSpeed.getDouble(0.25));
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
