// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
		TalonFXConfiguration config = new TalonFXConfiguration()
				.withMotorOutput(
						new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Coast))
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimit(Amps.of(120))
								.withStatorCurrentLimitEnable(true));

		// instantiate motor controllers
		for (int i = 0; i < shooterMotors.length; i++) {
			shooterMotors[i] = new TalonFX(ShooterConstants.SHOOTER_MOTORS[i]);
			shooterMotors[i].getConfigurator().apply(config);
		}

		uptakeMotor.getConfigurator().apply(config);

		shooterSpeed = Shuffleboard.getTab("testing").add("Shooter Motor Speed", .25).getEntry();
		uptakeSpeed = Shuffleboard.getTab("testing").add("Uptake Motor Speed", .25).getEntry();
		Shuffleboard.getTab("testing").add("Run Shooter", this.runShooter(() -> shooterSpeed.getDouble(0.25)));
		Shuffleboard.getTab("testing").add("Run Uptake", this.runUptake(() -> uptakeSpeed.getDouble(0.25)));
	}

	public Command runShooter(DoubleSupplier speed) {
		return run(() -> {
			for (TalonFX motor : shooterMotors) {
				motor.set(speed.getAsDouble());
			}
		}).finallyDo(() -> {
			for (TalonFX motor : shooterMotors) {
				motor.set(0);
			}
		});
	}

	public Command runUptake(DoubleSupplier speed) {
		return run(() -> uptakeMotor.set(speed.getAsDouble())).finallyDo(() -> uptakeMotor.set(0));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
