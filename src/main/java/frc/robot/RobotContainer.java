// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; //for sysid

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final FieldCentric drive = new FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final FieldCentricFacingAngle driveHeading = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(10, 0, 0);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double[] leftDeadbanded = AllDeadbands.applyCircularDeadband(new double[]{joystick.getLeftX(), joystick.getLeftY()}, .05);
                Rotation2d heading = getHeadingFromStick(() -> -joystick.getRightY(), () -> -joystick.getRightX());
                if(heading != null) {
                    return driveHeading.withVelocityX(leftDeadbanded[1] * MaxSpeed)
                        .withVelocityY(leftDeadbanded[0] * MaxSpeed)
                        .withTargetDirection(heading);
                }

                return drive.withVelocityX(leftDeadbanded[1] * MaxSpeed)
                    .withVelocityY(leftDeadbanded[0] * MaxSpeed)
                    .withRotationalRate((joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()) * MaxAngularRate);
            })
                
        );
        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        
        joystick.a().toggleOnTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Rotation2d getHeadingFromStick(DoubleSupplier rotationX, DoubleSupplier rotationY) {
        Rotation2d heading;
        double[] deadbandRotationInputs = AllDeadbands
                .applyCircularDeadband(new double[] { rotationX.getAsDouble(), rotationY.getAsDouble() }, 0.95);
        if (deadbandRotationInputs[0] != 0 || deadbandRotationInputs[1] != 0) {
            heading = Rotation2d.fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0]));
        } else {
            heading = null; //keep existing heading
        }
        return heading;
    }

    public Command getAutonomousCommand() {
        try{
            return autoChooser.getSelected();
        }
        catch(Exception e){
            System.out.println(e.toString());
            return null;
        }
    }
}
