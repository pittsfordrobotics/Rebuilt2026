package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.ShooterHelpers;
import frc.robot.lib.util.SwerveHelpers;
import frc.robot.lib.util.TalonConfigurator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
     private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private CommandXboxController controller;

    public final FieldCentric drive = new FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final FieldCentricFacingAngle driveHeading = new FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(10, 0, 0);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond);

    @Logged(name = "slowModeEnabled")
    private boolean slowModeEnabled;

    @Logged(name = "circleModeEnabled")
    private boolean circleModeEnabled;

    @Logged(name = "Current Command")
    public String SwerveCurrentCommand() {
        if (this.getCurrentCommand() == null) {
            return "none";
        }
        return this.getCurrentCommand().getName();
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        CommandXboxController controller,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        this.controller = controller;
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureMotorStatusFrames();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        CommandXboxController controller,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.controller = controller;
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureMotorStatusFrames();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        CommandXboxController controller,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        this.controller = controller;
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureMotorStatusFrames();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            ex.printStackTrace();
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }


    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
	
    public void addVisionMeasurement(VisionData visionData) {
        // this.resetPose(visionData.visionPose());
        super.addVisionMeasurement(visionData.visionPose(), Utils.fpgaToCurrentTime(visionData.time()), visionData.visionReliability());
    }

    // *******************
    // Logging methods
    // *******************
    @Logged(name = "Raw Rotation Degrees")
    public double getRotationDegrees() {
        return this.getState().RawHeading.getDegrees();
    }

    @Logged(name = "Pigeon Heading")
    public double getPigeonHeading() {
        return this.getPigeon2().getYaw().getValue().in(Degree);
    }

    @Logged(name = "Pose Heading")
    public double getPoseHeading() {
        return this.getState().Pose.getRotation().getDegrees();
    }

    @Logged(name = "FR Drive Motor")
    public TalonFX getFrontRightDriveMotor() {
        return this.getModule(1).getDriveMotor();
    }

    @Logged(name = "FR Steer Motor")
    public TalonFX getFrontRightSteerMotor() {
        return this.getModule(1).getSteerMotor();
    }

    @Logged(name = "FL Drive Motor")
    public TalonFX getFrontLeftDriveMotor() {
        return this.getModule(0).getDriveMotor();
    }

    @Logged(name = "FL Steer Motor")
    public TalonFX getFrontLeftSteerMotor() {
        return this.getModule(0).getSteerMotor();
    }

    @Logged(name = "BR Drive Motor")
    public TalonFX getBackRightDriveMotor() {
        return this.getModule(3).getDriveMotor();
    }

    @Logged(name = "BR Steer Motor")
    public TalonFX getBackRightSteerMotor() {
        return this.getModule(3).getSteerMotor();
    }

    @Logged(name = "BL Drive Motor")
    public TalonFX getBackLeftDriveMotor() {
        return this.getModule(2).getDriveMotor();
    }

    @Logged(name = "BL Steer Motor")
    public TalonFX getBackLeftAngleMotor() {
        return this.getModule(2).getSteerMotor();
    }

    public FieldCentricFacingAngle circleDrive() {
        Supplier<Translation2d> targetPoint = () -> getPlaceToShootAt();
        Translation2d currentPoint = this.getState().Pose.getTranslation();
        Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
        Translation2d resMath = circleDriveMath();
        return driveHeading.withVelocityX(resMath.getX())
            .withVelocityY(resMath.getY())
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withTargetDirection(targetHeading);
    }

    public Translation2d circleDriveMath() {
        Translation2d position = this.getState().Pose.getTranslation();
        Translation2d hubPosition = FieldConstants.flippedHubPosition.get();
        Translation3d position3d = new Translation3d(position);
        Translation3d hubPosition3d = new Translation3d(hubPosition);
        double deadbandedDirection = MathUtil.applyDeadband(controller.getLeftX(), .1);
        Translation2d perpDirection = new Translation3d(
            position3d.minus(hubPosition3d).cross(
                position3d.minus(hubPosition3d).plus(new Translation3d(0, 0, 1)))
            ).toTranslation2d();

        perpDirection = perpDirection.div(perpDirection.getNorm())
            .times(deadbandedDirection)
            .times(TunerConstants.CIRCLE_DRIVE_SPEED_MULTIPLIER);

        return perpDirection;
    }

    public Command drive() {
        // return this.applyRequest(() -> {
        //     // if(!circleModeEnabled) {
        //         double[] leftDeadbanded = SwerveHelpers.swerveDeadband(new double[]{controller.getLeftX(), controller.getLeftY()}, .1);
        //         Rotation2d heading = SwerveHelpers.getHeadingFromStick(() -> controller.getRightY(), () -> controller.getRightX());
        //         double adjustedMaxSpeed = slowModeEnabled ? MaxSpeed * TunerConstants.kSlowModePercent : MaxSpeed;
        //         double adjustedMaxAngularRate = slowModeEnabled ? RotationsPerSecond.of(2.5).in(RadiansPerSecond) * TunerConstants.kSlowModePercent :
        //         RotationsPerSecond.of(2.5).in(RadiansPerSecond);
        //         if(heading != null) {
        //             return driveHeading.withVelocityX(leftDeadbanded[1] * adjustedMaxSpeed)
        //                 .withVelocityY(leftDeadbanded[0] * adjustedMaxSpeed)
        //                 .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        //                 .withTargetDirection(AllianceFlipUtil.apply(heading));

        //         }
        //         return drive.withVelocityX(leftDeadbanded[1] * adjustedMaxSpeed)
        //             .withVelocityY(leftDeadbanded[0] * adjustedMaxSpeed)
        //             .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        //             .withRotationalRate((controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()) * adjustedMaxAngularRate);
        //     // }

        //     // return circleDrive();
        // });
        return this.applyRequest(() -> driveReq());
    }

    private SwerveRequest driveReq() {
        if(!circleModeEnabled) {
                double[] leftDeadbanded = SwerveHelpers.swerveDeadband(new double[]{controller.getLeftX(), controller.getLeftY()}, .1);
                Rotation2d heading = SwerveHelpers.getHeadingFromStick(() -> controller.getRightY(), () -> controller.getRightX());
                double adjustedMaxSpeed = slowModeEnabled ? MaxSpeed * TunerConstants.kSlowModePercent : MaxSpeed;
                double adjustedMaxAngularRate = slowModeEnabled ? RotationsPerSecond.of(2.5).in(RadiansPerSecond) * TunerConstants.kSlowModePercent :
                RotationsPerSecond.of(2.5).in(RadiansPerSecond);
                if(heading != null) {
                    return driveHeading.withVelocityX(leftDeadbanded[1] * adjustedMaxSpeed)
                        .withVelocityY(leftDeadbanded[0] * adjustedMaxSpeed)
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                        .withTargetDirection(AllianceFlipUtil.apply(heading));

                }
                return drive.withVelocityX(leftDeadbanded[1] * adjustedMaxSpeed)
                    .withVelocityY(leftDeadbanded[0] * adjustedMaxSpeed)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withRotationalRate((controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()) * adjustedMaxAngularRate);
            }

            return circleDrive();
    }

    public void enableSlowDrive() {
        slowModeEnabled = true;
    }
    public void disableSlowDrive() {
        slowModeEnabled = false;
    }

    public void enableCircleDrive() {
        circleModeEnabled = true;
    }
    public void disableCircleDrive() {
        circleModeEnabled = false;
    }
    
    public Command pointAt(Supplier<Translation2d> targetPoint) {
        return this.applyRequest(() -> {
                Translation2d currentPoint = this.getState().Pose.getTranslation();
                Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
                double[] leftDeadbanded = SwerveHelpers.swerveDeadband(new double[]{controller.getLeftX(), controller.getLeftY()}, .1);
                double adjustedMaxSpeed = slowModeEnabled ? MaxSpeed * TunerConstants.kSlowModePercent : MaxSpeed;
                return driveHeading.withVelocityX(leftDeadbanded[1] * adjustedMaxSpeed)
                    .withVelocityY(leftDeadbanded[0] * adjustedMaxSpeed)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withTargetDirection(targetHeading);
        });
    }

    public Command pointAtHub() {
        return this.pointAt(() -> getPlaceToShootAt());
    }

    @Logged(name="SOTM Target")
    public Translation2d getPlaceToShootAt() {
            ChassisSpeeds speeds = currChassisSpeeds();
            double velocityX = speeds.vxMetersPerSecond;
            double velocityY = speeds.vyMetersPerSecond;

            double speed = Math.sqrt(velocityX*velocityX + velocityY*velocityY);
            if(speed < .1) return FieldConstants.flippedHubPosition.get();

            double flightTime = estFlightTime(); //seconds

            double distX = velocityX * flightTime;
            double distY = velocityY * flightTime;

            Translation2d hubPos = FieldConstants.flippedHubPosition.get();
            return hubPos.minus(new Translation2d(distX, distY));

            // return FieldConstants.flippedHubPosition.get();
    }

    private double estFlightTime() {
        return (0.005926*ShooterHelpers.getHubDistInches(() -> this.getState().Pose)) + 0.5556;
    }

    public Command pointAtHubStatic() {
        return this.pointAt(FieldConstants.flippedHubPosition).until(() -> isPointedAtHub()).andThen(this.brake());
    }

    public boolean isPointedAtHub() {
        Supplier<Translation2d> targetPoint = FieldConstants.flippedHubPosition;
        Translation2d currentPoint = this.getState().Pose.getTranslation();
        Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
        Rotation2d currentHeading = this.getState().Pose.getRotation();
        return Math.abs(targetHeading.minus(currentHeading).getDegrees()) <= 2;
    }

    public Command brake() {
        return this.applyRequest(() -> brake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    
    public Command driveToPose(Supplier<Pose2d> targetPose) {
        PathConstraints constraints = new PathConstraints(
            TunerConstants.kSpeedAt12Volts, 
            MetersPerSecondPerSecond.of(1),
            DegreesPerSecond.of(180),
            DegreesPerSecondPerSecond.of(10));
        return Commands.defer(() -> AutoBuilder.pathfindToPose(targetPose.get(), constraints), Set.of(this));
    }

    public Command driveToPoint(Supplier<Translation2d> targetPoint) {
        return driveToPose(() -> new Pose2d(targetPoint.get(), this.getState().Pose.getRotation()));
    }

    public Command driveToAndPointAt(Supplier<Translation2d> targetPoint) {
        return driveToPose(() -> {
            Translation2d currentPoint = this.getState().Pose.getTranslation();
            Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
            return new Pose2d(targetPoint.get(), targetHeading);
        });
    }

    public Command driveToDistFromBlueHub(DoubleSupplier dist) {
        return this.driveToPoint(() ->
            new Translation2d(Units.inchesToMeters(-0.7536*dist.getAsDouble()+182.11),
            Units.inchesToMeters(-0.6574*dist.getAsDouble()+158.85))).andThen(this.pointAtHub());
    }

    public Command pointAtAllianceZone() {
        // return this.pointAt(() -> AllianceFlipUtil.apply(
        //     new Translation2d(0, this.getState().Pose.getY())
        // ));

        return this.pointAt(() -> new Translation2d(AllianceFlipUtil.flipX(0), this.getState().Pose.getY()));
    }

    private void configureMotorStatusFrames() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : this.getModules()) {
            TalonConfigurator.reduceCommonStatusFrameFrequencies(module.getDriveMotor());
            TalonConfigurator.reduceCommonStatusFrameFrequencies(module.getSteerMotor());
        }
    }

    private ChassisSpeeds currChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().Speeds, this.getState().RawHeading);
    }

    // @Logged(name="Velocity") //don't really want to clog up logs
    public Translation2d getVelocity() {
        ChassisSpeeds speeds = currChassisSpeeds();
        return this.getState().Pose.getTranslation().plus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    public BooleanSupplier isInCircleMode() {
        return () -> circleModeEnabled;
    }
}
