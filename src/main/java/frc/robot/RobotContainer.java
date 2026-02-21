// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; //for sysid

import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final Vision vision;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    private GenericEntry testingDistToHub;


    @Logged(name = "Swerve")
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(driverController);

    @Logged(name = "PDH")
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final Intake intake;
    private final Indexer indexer;

    @Logged(name="Shooter")
    private final Shooter shooter;
    private final Climber climber;

    public RobotContainer() {
	    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
		
        vision = new Vision(
            () -> drivetrain.getState().RawHeading,
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond,
            drivetrain::addVisionMeasurement,
            // VisionConstants.LIMELIGHT_LEFT,
            // VisionConstants.LIMELIGHT_RIGHT,
            VisionConstants.LIMELIGHT_FRONT);

        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        intake= new Intake();
        shooter = new Shooter();
        indexer = new Indexer();
        climber = new Climber();

        configureBindings();
        testingShuffleboardInit();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.drive());
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        driverController.a().toggleOnTrue(drivetrain.brake());
        driverController.b().whileTrue(drivetrain.pointAtHub());
        driverController.x().whileTrue(drivetrain.driveToPoint(FieldConstants.flippedHubPosition));
        driverController.x().and(driverController.b()).whileTrue(drivetrain.driveToAndPointAt(FieldConstants.flippedHubPosition));

        operatorController.rightBumper().whileTrue(shooter.runShooter());
        operatorController.leftBumper().whileTrue(indexer.runIndex());
        operatorController.b().whileTrue(Commands.parallel(shooter.shootAtHub(() -> drivetrain.getState().Pose), indexer.runIndex(), drivetrain.pointAtHub()));
        operatorController.y().whileTrue(climbUp());
        operatorController.y().whileFalse(climber.runClimber(() -> -0.05));
        operatorController.x().whileTrue(climbDown());
        operatorController.a().whileTrue(intake.runIntake());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(
            () -> drivetrain.resetRotation(AllianceFlipUtil.isRed() ? Rotation2d.k180deg : Rotation2d.kZero)));

        // driverController.leftBumper().onTrue(drivetrain.runOnce(
        //     () -> drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)))
        // ));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void testingShuffleboardInit(){
        testingDistToHub = Shuffleboard.getTab("testing").add("Testing Dist to Hub", 100).getEntry();
        Shuffleboard.getTab("testing").add("Drive to testing point", drivetrain.driveToDistFromBlueHub(() -> testingDistToHub.getDouble(0)));
    }
    

    public Command getAutonomousCommand() {
        try {
            return autoChooser.getSelected();
        } catch(Exception e) {
            System.out.println(e.toString());
            return null;
        }
    }

    public Command climbDown() {
       return climber.runClimber(() -> 0.4);
    }

    public Command climbUp(){
        return drivetrain.driveToPose(ClimberConstants.FLIPPED_CLIMB_UNEXTENDED_POS)
            .alongWith(climber.runClimber(() -> 0), intake.pivotIn())
            .andThen(drivetrain.driveToPose(ClimberConstants.FLIPPED_CLIMB_EXTENDED_POS))
            .andThen(climber.runClimber(() -> -0.4));
    }
}
