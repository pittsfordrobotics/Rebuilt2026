// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
	/** Creates a new Shooter. */

	private GenericEntry shooterSpeed;
	private GenericEntry uptakeSpeed;
	private GenericEntry hoodPercent;
	private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

	public final TalonFX[] shooterMotors = new TalonFX[ShooterConstants.SHOOTER_MOTORS.length];

    @Logged(name="Uptake Motor")
	final TalonFX uptakeMotor = new TalonFX(ShooterConstants.UPTAKE_MOTOR);

	@Logged(name="Hood Actuator")
	final Servo hood = new Servo(ShooterConstants.HOOD_ACTUATOR);

	public Shooter() {
		TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Coast))
				.withVoltage(
					new VoltageConfigs()
						.withPeakReverseVoltage(Volts.of(0))
            	)
				.withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true))
				.withSlot0(
                	new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / ShooterConstants.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
				);

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

        shooterSpeed = Shuffleboard.getTab("testing").add("Shooter Motor Speed", .6).getEntry();
		uptakeSpeed = Shuffleboard.getTab("testing").add("Uptake Motor Speed", .6).getEntry();
		hoodPercent = Shuffleboard.getTab("testing").add("Hood Pos Percentage", 0.5).getEntry();
		Shuffleboard.getTab("testing").add("Set Hood Pos", this.runHood(() -> hoodPercent.getDouble(0.5)));
	}

	public Command runShooter(DoubleSupplier shootSpeed, DoubleSupplier uptakeSpeed) {
		return run(() -> {
			for (TalonFX motor : shooterMotors) {
				motor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed.getAsDouble()*(ShooterConstants.kFreeSpeed.in(RotationsPerSecond)*60))));
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

    @Logged(name="Middle Motor")
    public TalonFX getMiddleMotor() {
        return this.shooterMotors[0];
    }

	public Command runHood(DoubleSupplier position) {
		return run(() -> hood.set(position.getAsDouble()));
	}
}
