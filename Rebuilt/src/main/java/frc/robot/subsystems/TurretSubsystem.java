package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants.ImportantFieldPoses;
import frc.robot.Constants.RobotConstants.TurretConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;

/**
 * Subsystem controlling the turret mechanism with field-relative tracking.
 * 
 * <p>Features:
 * <ul>
 *   <li>Field-relative aiming with velocity feedforward</li>
 *   <li>Automatic compensation for robot rotation</li>
 *   <li>Intelligent angle wrapping for shortest path</li>
 *   <li>Smooth motion profiling</li>
 *   <li>Dynamic target switching (Hub, Ferry, Custom positions)</li>
 *   <li>Alliance-aware targeting for FRC 2026 Reefscape</li>
 * </ul>
 * 
 * <p>Physical Constraints:
 * <ul>
 *   <li>Range: -135° to +135° (270° total)</li>
 *   <li>Gear Ratio: {@link TurretConstants#DEGREE_CHANGE_PER_MOTOR_REVOLUTION} degrees per motor revolution</li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {
    
    // ========================= Hardware =========================
    
    private static volatile TurretSubsystem INSTANCE;
    
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;
    private final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    
    private CommandXboxController driverController;
    private double controllersCurrentRumbling = 0.0;
    
    // ========================= Configuration =========================
    
    // Physical limits (degrees) - degree° total range depends on the constant
    private static final double MIN_ANGLE_DEG = -(TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    private static final double MAX_ANGLE_DEG = (TurretConstants.TOTAL_ROTATION_DEGREES / 2);
    
    // Conversion constants
    private final double angleOffsetToZero = TurretConstants.TURRET_OFFSET_TO_ZERO;
    private final double degreeOffsetToTarget = TurretConstants.TURRET_OFFSET_TO_TARGET;
    private final double encoderToAngleMultiplier = TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
    
    private final double allowedEncoderDiff = 0.05;
    
    // Tunable parameters
    private final LoggedNetworkNumber logMaxVelocityDegPerSec = 
        new LoggedNetworkNumber("/Tuning/Turret/MaxVelocityDegPerSec", 450); //Based off how long it takes to wrap
    private final LoggedNetworkNumber logMaxAccelerationDegPerSecSq = 
        new LoggedNetworkNumber("/Tuning/Turret/MaxAccelerationDegPerSecSq", 360);
    
    private final LoggedNetworkNumber logKP = new LoggedNetworkNumber("/Tuning/Turret/kP", 50); // 62.5
    private final LoggedNetworkNumber logKI = new LoggedNetworkNumber("/Tuning/Turret/kI", 0.0);
    private final LoggedNetworkNumber logKD = new LoggedNetworkNumber("/Tuning/Turret/kD", 0.0);
    private final LoggedNetworkNumber logKS = new LoggedNetworkNumber("/Tuning/Turret/kS", 0.21);
    private final LoggedNetworkNumber logKV = new LoggedNetworkNumber("/Tuning/Turret/kV",0.0115);
    private final LoggedNetworkNumber logKA = new LoggedNetworkNumber("/Tuning/Turret/kA", 0);

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Turret", false);
    private boolean goodToMove = true;

    // Cached values
    private double cachedAngleDeg = 0.0;
    private double cachedMotorVelocity = 0.0;
    private double cachedMaxVelocity = logMaxVelocityDegPerSec.get();

    // Motor configuration
    private TalonFXConfiguration motorConfig;
    
    // ========================= Control =========================
    
    /**
     * Turret operating states.
     */
    public enum TurretState {
        /** Do nothing, sit there and look pretty */
        DISABLED,
        /** Goes to store, then once there, goes to aiming */
        INIT,
        /** Holding current position, no active tracking */
        AIMING,
        /** Moving to safe storage position (0°) */
        STORING,

        /**  If hood flips off */
        EMERGENCY,

        CLIMB
    }
    
    /**
     * Pre-defined targeting modes for common game actions.
     */
    public enum TargetMode {
        /** Aiming at alliance Hub for shooting */
        HUB,
        /** Aiming at alliance hub with SOTM Implementation */
        ADJ_HUB,
        /** Aiming at ferry target for passing */
        FERRY,
        /** Aiming at custom position set via setCustomTarget() */
        CUSTOM
    }
    
    private TurretState currentState = TurretState.AIMING;
    private TurretState previousState = currentState;
    
    // Target management
    private TargetMode currentTargetMode = TargetMode.ADJ_HUB;
    private Translation2d customTargetPosition = null;
    
    // Field-relative targeting
    private Rotation2d fieldRelativeGoalAngle = Rotation2d.fromDegrees(0);
    
    // Robot-relative targeting (calculated each cycle)
    private double robotRelativeTargetAngleDeg = 0.0;
    private double robotRelativeTargetVelocityDegPerSec = 0.0;
    /**
     * Used for avegering velocity, unused at the current moment.
     */
    @SuppressWarnings("unused")
    private double previousRobotRelativeTargetVelocityDegPerSec = 0.0;
    private double previousRobotRelativeTargetAngleDeg = 0.0;
    private double degreesToStopWithCurrentVelocity = 0.0;

    private Rotation2d previousTurretAngleWRTField = new Rotation2d();

    private double previousAnglesTimestamp = 0.0;
    private double previousVelocityTimestamp = 0.0;
    
    // State tracking
    private double turretOffset = 0.0;
    
    // Motion profiling
    private SlewRateLimiter velocityLimiter;
    private double currentRateLimit;
    private double profiledSetpointDeg = 0.0;

    private boolean useManualAdjust = true;
    private double manualShift = 0;
    
    // Control requests
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(false);
    private final SimpleMotorFeedforward feedforward;

    //Testing Stuff
    public double testTargetSetpoint = 0;
    
    // ========================= Constructor =========================
    
    /**
     * Creates a new TurretSubsystem.
     */
    public TurretSubsystem() {
        
        // Initialize hardware
        turretMotor = new TalonFX(
            TurretConstants.DeviceIDs.kMotorID, 
            RobotConstants.MAIN_SYSTEMS_CANBUS
        );

        turretEncoder = new CANcoder(
            TurretConstants.DeviceIDs.kAbsEncoderID, 
            RobotConstants.MAIN_SYSTEMS_CANBUS
        );
        
        // Configure hardware
        configureEncoder();
        configureMotor();
        
        // Initialize feedforward controller
        feedforward = new SimpleMotorFeedforward(
            logKS.get(), 
            logKV.get(), 
            logKA.get()
        );
        
        // Initialize motion profiler
        velocityLimiter = new SlewRateLimiter(
            Math.toRadians(logMaxAccelerationDegPerSecSq.get())
        );
        currentRateLimit = Math.toRadians(logMaxAccelerationDegPerSecSq.get());

        drivetrain.getPigeon2().getAngularVelocityZDevice().setUpdateFrequency(50);
    }
    
    // ========================= Configuration =========================
    
    /**
     * Configures the Encoder <br><br/>
     * Runs upon instance construction <br><br/>
     * Configures:
     *  <ul>
     *  <li> Rotation Direction
     *  <li> Encoder Magnet Offset
     *  <ul/>
     */
    private void configureEncoder() {
        //IRL
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_MAGNET_OFFSET;
        
        turretEncoder.getConfigurator().apply(encoderConfig);
    }
    
    /**
     * Configures the motor <br><br/>
     * Runs upon instance construction, and constant change (tuning mode) <br><br/>
     * Configures:
     *  <ul>
     *  <li> Neutral Mode Value
     *  <li> Rotation Direction
     *  <li> Software Limits
     *  <li> Stator & Supply Current Limits
     *  <li> Peak Motor Voltage
     *  <li> Slot 0 Constants
     *  <li> Remote Sensor Type & Ratios
     *  <li> Peak Tourqe current
     *  <ul/>
     */
    private void configureMotor() {
        //IRL
        motorConfig = new TalonFXConfiguration();
        
        // Neutral mode
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Calculate motor revolutions for your angle limits
        double minMotorRevs = MIN_ANGLE_DEG / TurretConstants.ABS_ENCDR_POS_TO_ANGLE;
        double maxMotorRevs = MAX_ANGLE_DEG / TurretConstants.ABS_ENCDR_POS_TO_ANGLE;

        // Now account for the rotor-to-sensor ratio
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxMotorRevs;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minMotorRevs;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 15;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        motorConfig.Voltage.PeakForwardVoltage = 7;
        motorConfig.Voltage.PeakReverseVoltage = -7;
        
        // PID gains (Slot 0)
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(logKP.get())
            .withKI(logKI.get())
            .withKD(logKD.get())
            .withKS(logKS.get())
            .withKV(logKV.get())
            .withKA(logKA.get());
        
        motorConfig.Slot0 = slot0;
        
        // Fused CANcoder feedback
        motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_SENSOR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        
        // Apply configuration
        turretMotor.getConfigurator().apply(motorConfig);

        // syncMotorPosToEncoder();
    }
    
    // ========================= Periodic =========================
    
    @Override
    public void periodic() {
        // cachedAngleDeg = turretMotor.getPosition().getValueAsDouble() / TurretConstants.ROTOR_TO_SENSOR_RATIO * TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION + turretOffset;
        cachedAngleDeg = turretEncoder.getAbsolutePosition().getValueAsDouble() / TurretConstants.SENSOR_TO_MECHANISM_RATIO * encoderToAngleMultiplier + turretOffset;
        cachedMotorVelocity = turretMotor.getVelocity().getValueAsDouble();

        if (Math.abs(turretMotor.getPosition().getValueAsDouble()) > (MAX_ANGLE_DEG + 5) / TurretConstants.ABS_ENCDR_POS_TO_ANGLE && CoordinationSubsystem.getInstance().getCurrentAbsoluteState() == AbsoluteStates.TRENCH) {
            goodToMove = false;
        } else {
            goodToMove = true;
        }

        if (Math.abs(turretEncoder.getAbsolutePosition().getValueAsDouble() - turretEncoder.getPosition().getValueAsDouble()) > allowedEncoderDiff) {
            resetEncodersRelativePosition();
        }
    
        // Update target based on current mode
        updateTargetFromMode();
        
        // Convert field-relative goal to robot-relative 
        Rotation2d robotRelativeGoalAngle = fieldRelativeGoalAngle.plus(
            new Rotation2d(Units.degreesToRadians(angleOffsetToZero + degreeOffsetToTarget)));
        
        // Find best angle considering wrapping and limits
        robotRelativeTargetAngleDeg = findBestWrappedAngle(robotRelativeGoalAngle.getDegrees());

        // Convert RPS to degrees/sec
        double currentVelocityDegPerSec = cachedMotorVelocity 
            * TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION;
        // Logger.recordOutput("Rebuilt/Turret/Pos/CurrentVelocityDegPerSec", currentVelocityDegPerSec);

        // Calculate stopping distance: v² / (2a)
        double maxDeceleration = currentRateLimit;
        degreesToStopWithCurrentVelocity = (Math.pow(currentVelocityDegPerSec, 2) 
            / (2 * maxDeceleration));

        previousRobotRelativeTargetVelocityDegPerSec = robotRelativeTargetVelocityDegPerSec;
        // robotRelativeTargetVelocityDegPerSec = calculateVelocity();
        robotRelativeTargetVelocityDegPerSec = calculateVelocityUpdated();
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Velocity WRT Robot", calculateVelocityUpdated());

        // Driver feedback for out-of-range targets
        updateDriverFeedback();
        
        // Apply state machine
        applyState();
        
        // Publish telemetry
        publishLogs();
    }

    /**
     * Calculates the target Velocity in Degrees per Second.
     * <p>Safety Precautions:
     * <ul>
     *   <li>If at deadzone, do not use any velocity</li>
     *   <li>If unclamped velocity will lead to going outside of deadzone, limit velocity to what will not go outside of deadzone</li>
     * </ul>
     * @return the target velocity
     */
    private double calculateVelocity() {
        boolean usedVeloc = false;
        double targetVeloc = 0;

        double robotYawVeloc = drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble();
        // Logger.recordOutput("Rebuilt/Turret/Robot Yaw Veloc", robotYawVeloc);

        if ((robotRelativeTargetAngleDeg <= MIN_ANGLE_DEG || robotRelativeTargetAngleDeg >= MAX_ANGLE_DEG) || (cachedAngleDeg <= MIN_ANGLE_DEG || cachedAngleDeg >= MAX_ANGLE_DEG)) {
            targetVeloc = 0.0;
        } else {
            usedVeloc = true;
            boolean isVelocityPositive = cachedMotorVelocity > 0;
            boolean allowVeloc = false;
            double maxVelocity = 0.0;
            double currentAngle = cachedAngleDeg;
            if (isVelocityPositive) {
                // Check if we can continue moving positive without hitting max limit
                if (currentAngle + degreesToStopWithCurrentVelocity < MAX_ANGLE_DEG) {
                    allowVeloc = true;
                    maxVelocity = Math.abs(currentAngle - MAX_ANGLE_DEG);// / (Timer.getFPGATimestamp() - previousAnglesTimestamp));
                }
            } else {
                // Check if we can continue moving negative without hitting min limit
                if (currentAngle - degreesToStopWithCurrentVelocity > MIN_ANGLE_DEG) {
                    allowVeloc = true;
                    maxVelocity = Math.abs(currentAngle - MIN_ANGLE_DEG);// / (Timer.getFPGATimestamp() - previousAnglesTimestamp)); // Distance to min limit
                }
            }
            if (allowVeloc) {
                // Calculate desired velocity (derivative of target position)
                double desiredVelocity = (robotRelativeTargetAngleDeg - previousRobotRelativeTargetAngleDeg) / (Timer.getFPGATimestamp() - previousAnglesTimestamp);
                
                previousRobotRelativeTargetAngleDeg = robotRelativeTargetAngleDeg;
                previousAnglesTimestamp = Timer.getFPGATimestamp();

                //Avg veloc
                // desiredVelocity = (desiredVelocity + previousRobotRelativeTargetVelocityDegPerSec) / 2;
                
                if (Math.abs(desiredVelocity) > 0.175) {
                    targetVeloc = desiredVelocity;
                    // targetVeloc = MathUtil.clamp(desiredVelocity, -maxVelocity, maxVelocity); //if clamping it breaks it, then set it to desiredVelocity
                } else {
                    targetVeloc = 0.0;
                }
                

                // if(targetVeloc != desiredVelocity) {
                //     System.out.println("CANT KEEP UP");
                //     double angleWentTo = targetVeloc * (time);
                //     double angleWentToRad = angleWentTo * Math.PI/180;
                //     if(Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 1) {
                //         RobotContainer.maxAngularRate = targetVeloc;
                //     }
                //     else if(Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) > .1) {
                //         RobotContainer.maxSpeed = Math.abs(((drivetrain.getState().Speeds.vyMetersPerSecond)/(Math.tan(angleWentToRad))));
                //     }
                // }
            }
            // Logger.recordOutput("Rebuilt/Turret/Veloc Net/MaxVelocity", maxVelocity);
        }
        // Logger.recordOutput("Rebuilt/Turret/Veloc Net/Used Velocity", usedVeloc);
        // Logger.recordOutput("Rebuilt/Turret/Veloc Net/Calculated Veloc", targetVeloc);
        // Logger.recordOutput("Rebuilt/Turret/Combined Veloc", targetVeloc + robotYawVeloc);
        return targetVeloc;
    }   


    public double calculateVelocityUpdated() {
        if ((robotRelativeTargetAngleDeg <= MIN_ANGLE_DEG || robotRelativeTargetAngleDeg >= MAX_ANGLE_DEG) || (cachedAngleDeg <= MIN_ANGLE_DEG || cachedAngleDeg >= MAX_ANGLE_DEG)) return 0.0;

        double turretVeloictyWRTRobot = 0.0;
        double robotYawVelocWRTField = drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble(); //degrees/second

        Rotation2d robotYaw = drivetrain.getState().Pose.getRotation(); //rotation
        Rotation2d turretAngleWRTField = robotYaw.plus(new Rotation2d(Units.degreesToRadians(robotRelativeTargetAngleDeg))); //radians

        double timestamp = Timer.getFPGATimestamp(); //seconds

        Rotation2d deltaTurretAngleWRTField = turretAngleWRTField.minus(previousTurretAngleWRTField);

        // Calculate desired velocity (derivative of target position)
        double turretVelocityWRTField = (deltaTurretAngleWRTField.getDegrees()) / (timestamp - previousVelocityTimestamp); //degrees/second

        turretVeloictyWRTRobot = turretVelocityWRTField - robotYawVelocWRTField; //degrees/second
    
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Previous Angle WRT Field", previousTurretAngleWRTField);
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Previous Timestamp", previousVelocityTimestamp);

        previousTurretAngleWRTField = turretAngleWRTField;
        previousVelocityTimestamp = timestamp;

        // Logger.recordOutput("Rebuilt/Turret/Velocity/Velocity WRT Field", turretVelocityWRTField);
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Current Timestamp", timestamp);
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Robot Angular Rate", robotYawVelocWRTField);
        // Logger.recordOutput("Rebuilt/Turret/Velocity/Turret Angle WRT Field", turretAngleWRTField);

        return turretVeloictyWRTRobot;
    }
    
    /**
     * Updates the field-relative goal based on the current target mode.
     * Called every periodic cycle to recalculate aim point.
     */
    private void updateTargetFromMode() {
        Translation2d targetPosition = getCurrentTargetPosition();
        
        if (targetPosition != null) {
            
            // Calculate vector from robot to target
            Translation2d robotToTarget = targetPosition;
            
            // Calculate angle to target
            Rotation2d angleToTarget = new Rotation2d(
                robotToTarget.getX(), 
                robotToTarget.getY()
            );

            
            // Update field-relative goal
            fieldRelativeGoalAngle = angleToTarget;
        }
    }

    public Rotation2d updateTargetFromPose(Translation2d targetPosition) {

        if (targetPosition != null) {
            // Calculate vector from robot to target
            Translation2d robotToTarget = targetPosition;
            
            // Calculate angle to target
            Rotation2d angleToTarget = new Rotation2d(
                robotToTarget.getX(), 
                robotToTarget.getY()
            );

            
            // Update field-relative goal (target is stationary, so velocity = 0)
            return angleToTarget;
        }

        return null;
    }
    
    /**
     * Gets the current target position based on active targeting mode.
     * 
     * @return Target position on field, or null if no valid target
     */
    private Translation2d getCurrentTargetPosition() {
        return switch (currentTargetMode) {
            case HUB -> FieldConstants.isBlueAlliance 
                ?  ImportantFieldPoses.BLUE_HUB_CENTER
                :  ImportantFieldPoses.RED_HUB_CENTER;

            case ADJ_HUB -> VisionSubsystem.getInstance().getAdjustedCurrentTarget();
            
            case FERRY -> FieldConstants.isBlueAlliance 
                ? drivetrain.getState().Pose.getY() > 4.0345
                    ? ImportantFieldPoses.BLUE_LEFT_FERRY_TARGET
                    : ImportantFieldPoses.BLUE_RIGHT_FERRY_TARGET
                : drivetrain.getState().Pose.getY() > 4.0345
                    ? ImportantFieldPoses.RED_LEFT_FERRY_TARGET
                    : ImportantFieldPoses.RED_RIGHT_FERRY_TARGET;
            
            case CUSTOM -> customTargetPosition;
        };
    }
    
    /**
     * State machine implementation.
     */
    private void applyState() {
        if (goodToMove) {
            switch (currentState) {
                case DISABLED:
                    turretMotor.stopMotor();
                    break;

                case INIT:
                    if (isAtProfiledGoal(20) && ShooterSubsystem.getInstance().getCurrentAngleState() != ShooterSubsystem.AngleState.ZEROING) {
                        currentState = TurretState.AIMING;
                    }

                    profiledSetpointDeg = 0;

                    profiledSetpointDeg = MathUtil.clamp(
                        profiledSetpointDeg, 
                        MIN_ANGLE_DEG, 
                        MAX_ANGLE_DEG
                    );
                    
                    commandTurret(profiledSetpointDeg, 0.0, 0.0);
                    break;
                
                case EMERGENCY:
                    profiledSetpointDeg = 90;
                    commandTurret(profiledSetpointDeg, 0.0, 0.0);
                    break;

                case AIMING:
                    // Track target with velocity feedforward
                    double maxVelocity = cachedMaxVelocity;
                    
                    // Apply motion profiling for smooth acceleration
                    // profiledSetpointDeg = Math.toDegrees(
                    //     velocityLimiter.calculate(Math.toRadians(robotRelativeTargetAngleDeg)) //used to be robotRelativeTargetAngle
                    // );

                    profiledSetpointDeg = robotRelativeTargetAngleDeg;
                    
                    if(useManualAdjust) {
                        profiledSetpointDeg += manualShift;
                    }
                    // Clamp to physical limits
                    profiledSetpointDeg = MathUtil.clamp(
                        profiledSetpointDeg, 
                        MIN_ANGLE_DEG, 
                        MAX_ANGLE_DEG
                    );
                    
                    // Clamp velocity for safety
                    // double clampedVelocity = MathUtil.clamp(
                    //     robotRelativeTargetVelocityDegPerSec,
                    //     -maxVelocity,
                    //     maxVelocity
                    // );
                    double clampedVelocity = robotRelativeTargetVelocityDegPerSec;
                    
                    // Calculate acceleration for feedforward (derivative of velocity setpoint)
                    // For now, assume 0 unless tracking accelerating target
                    double accelerationDegPerSecSq = 0.0;
                    
                    commandTurret(profiledSetpointDeg, clampedVelocity, accelerationDegPerSecSq);
                    break;
                    
                case STORING:
                    // Return to zero position with no velocity
                    profiledSetpointDeg = 0;

                    profiledSetpointDeg = MathUtil.clamp(
                        profiledSetpointDeg, 
                        MIN_ANGLE_DEG, 
                        MAX_ANGLE_DEG
                    );

                    commandTurret(profiledSetpointDeg, 0.0, 0.0);

                    break;
                case CLIMB:
                    profiledSetpointDeg = 90;
                    profiledSetpointDeg = MathUtil.clamp(
                        profiledSetpointDeg, 
                        MIN_ANGLE_DEG, 
                        MAX_ANGLE_DEG
                    );
                    
                    commandTurret(profiledSetpointDeg, 0.0, 0.0);
            }
        } else {
            turretMotor.stopMotor();
        }
    }
    
    /**
     * Commands the turret with position, velocity, and acceleration feedforward.
     * 
     * @param positionDeg Target position in degrees (robot-relative)
     * @param velocityDegPerSec Target velocity in deg/s (robot-relative)
     * @param accelerationDegPerSecSq Target acceleration in deg/s² (robot-relative)
     */
    private void commandTurret(double positionDeg, double velocityDegPerSec, double accelerationDegPerSecSq) {

        positionDeg = MathUtil.clamp(
                    positionDeg, 
                    MIN_ANGLE_DEG, 
                    MAX_ANGLE_DEG
                );

        // Convert degrees to motor rotations
        double positionMotorRevs = positionDeg;
        double velocityMotorRevsPerSec = velocityDegPerSec;
        double accelerationMotorRevsPerSecSq = accelerationDegPerSecSq;
        
        // Convert to encoder units (accounting for offset and encoder ratio)
        double positionEncoderRevs = 
            (positionMotorRevs) / encoderToAngleMultiplier;
        
        // Calculate feedforward voltage
        // FF = kS*sgn(v) + kV*v + kA*a
        double feedforwardVolts = feedforward.calculate(
            velocityMotorRevsPerSec,
            accelerationMotorRevsPerSecSq
        );
        
        // Send position command with feedforward
        turretMotor.setControl(
            positionRequest
                .withPosition(positionEncoderRevs - (turretOffset / encoderToAngleMultiplier))
                .withFeedForward(feedforwardVolts)
        );
    }
    
    // ========================= Helper Methods =========================
    
    /**
     * Finds the best equivalent angle considering wrapping and physical limits.
     * Chooses the angle that requires the least turret rotation.
     * 
     * <p>With 270° range (-135° to +135°), the turret can nearly complete a full rotation,
     * so we check multiple wrapped versions to find the shortest path.
     * 
     * @param targetDegrees Target angle in degrees (robot-relative)
     * @return Best wrapped angle within [-135°, +135°]
     */
    private double findBestWrappedAngle(double targetDegrees) {
        double currentAngle = cachedAngleDeg;
        
        double bestAngle = targetDegrees;
        double minDistance = Double.POSITIVE_INFINITY;
        
        // Check wrapped versions: -720°, -360°, 0°, +360°, +720°
        // With 270° range, wrapping becomes more important
        for (int wraps = -2; wraps <= 2; wraps++) {
            double wrappedAngle = targetDegrees + (wraps * 360.0);
            
            // Skip if outside physical limits
            if (wrappedAngle < MIN_ANGLE_DEG || wrappedAngle > MAX_ANGLE_DEG) {
                continue;
            }
            
            // Calculate travel distance for this option
            double distance = Math.abs(currentAngle - wrappedAngle);
            
            // Update if this is the shortest path found so far
            if (distance < minDistance) {
                bestAngle = wrappedAngle;
                minDistance = distance;
            }
        }

        // If no valid angle found (all out of range), clamp to nearest limit
        if (minDistance == Double.POSITIVE_INFINITY) {
            bestAngle = MathUtil.clamp(targetDegrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        }
        
        return bestAngle;
    }
    
    /**
     * Checks if the current target is outside the turret's physical range.
     * 
     * @return true if target cannot be reached without robot rotation
     */
    private boolean isTargetOutOfRange() {
        return robotRelativeTargetAngleDeg <= MIN_ANGLE_DEG || 
               robotRelativeTargetAngleDeg >= MAX_ANGLE_DEG;
    }
    
    /**
     * Updates tunable parameters if they've changed.
     */
    public void updateTunableParameters() {
        boolean gainsChanged = false;
        boolean profileChanged = false;
        
        // Check if PID/FF gains changed
        if (motorConfig.Slot0.kP != logKP.get() ||
            motorConfig.Slot0.kI != logKI.get() ||
            motorConfig.Slot0.kD != logKD.get() ||
            motorConfig.Slot0.kS != logKS.get() ||
            motorConfig.Slot0.kV != logKV.get() ||
            motorConfig.Slot0.kA != logKA.get()) {
            
            gainsChanged = true;
        }
        
        // Check if profile parameters changed
        double newRateLimit = Math.toRadians(logMaxAccelerationDegPerSecSq.get());
        if (currentRateLimit != newRateLimit) {
            profileChanged = true;
            currentRateLimit = newRateLimit;
        }
        
        // Reconfigure if needed
        if (gainsChanged) {
            configureMotor();

            feedforward.setKa(logKA.get());
            feedforward.setKs(logKS.get());
            feedforward.setKv(logKV.get());
        }
        
        if (profileChanged) {
            velocityLimiter = new SlewRateLimiter(
                Math.toRadians(logMaxAccelerationDegPerSecSq.get())
            );
        }

        cachedMaxVelocity = logMaxVelocityDegPerSec.get();
    }
    
    /**
     * Provides rumble feedback when target is out of range.
     */
    private void updateDriverFeedback() {
        if (driverController != null) {
            if (currentState == TurretState.AIMING && isTargetOutOfRange() && DriverStation.isEnabled() && !DriverStation.isAutonomous()) {
                if (CoordinationSubsystem.getInstance().getInDeadzone() && CoordinationSubsystem.getInstance().getOverridingTrench()) {
                    if (controllersCurrentRumbling != 0.15) {
                        controllersCurrentRumbling = 0.15;
                        driverController.setRumble(RumbleType.kBothRumble, 0.15);
                    }
                } else if (profiledSetpointDeg == MIN_ANGLE_DEG) {
                    if (controllersCurrentRumbling != 0.05) {
                        controllersCurrentRumbling = 0.05;
                        driverController.setRumble(RumbleType.kLeftRumble, 0.15);
                    }
                } else if (profiledSetpointDeg == MAX_ANGLE_DEG) {
                    if (controllersCurrentRumbling != 0.1) {
                        controllersCurrentRumbling = 0.1;
                        driverController.setRumble(RumbleType.kRightRumble, 0.15);
                    }
                } else {
                    if (controllersCurrentRumbling != 0.0) {
                        controllersCurrentRumbling = 0.0;
                        driverController.setRumble(RumbleType.kBothRumble, 0.0);
                    }
                }
            } else {
                if (controllersCurrentRumbling != 0.0) {
                    controllersCurrentRumbling = 0.0;
                    driverController.setRumble(RumbleType.kBothRumble, 0.0);
                }
            }
        }
    }
    
    // ========================= Logging =========================
    
    /**
     * Publishes telemetry data to AdvantageKit Logger.
     */
    private void publishLogs() {
        

        // State information
        Logger.recordOutput("Rebuilt/Turret/State/CurrentState", currentState);
        // Logger.recordOutput("Rebuilt/Turret/State/TargetMode", currentTargetMode);
        
        // Position tracking
        Logger.recordOutput("Rebuilt/Turret/Pos/CurrentAngleDeg", cachedAngleDeg);
        Logger.recordOutput("Rebuilt/Turret/Target/ProfiledSetpointDeg", profiledSetpointDeg);

        Logger.recordOutput("Rebuilt/Turret/Device/Motor Supply Amps", turretMotor.getSupplyCurrent().getValueAsDouble());
        
        // Velocity tracking
        // Logger.recordOutput("Rebuilt/Turret/Pos/CurrentVelocityDegPerSec", 
            // cachedMotorVelocity * TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION);
        // Logger.recordOutput("Rebuilt/Turret/Target/TargetVelocityDegPerSec", 
        //     robotRelativeTargetVelocityDegPerSec);

        //Debugging For Creating Velocity Safety Net
        // Logger.recordOutput("Rebuilt/Turret/Pos/Distance to Stop", degreesToStopWithCurrentVelocity);

        Logger.recordOutput("Rebuilt/Turret/Pos/Manual Adjustment", manualShift);

        if (logDebuggingToggle.get()) {

            Pose2d currentBotPose = drivetrain.getState().Pose;
            Pose2d currentBotPoseWRTTurret = currentBotPose.transformBy(TurretConstants.OFFSET_WRT_CENTER_OF_ROBOT);
            Translation3d turretPoseTranslation = new Translation3d(currentBotPoseWRTTurret.getX(), currentBotPoseWRTTurret.getY(), 0.25);
            Rotation3d turretPoseRotation = new Rotation3d(0, 0, fieldRelativeGoalAngle.getRadians());
            Logger.recordOutput("Rebuilt/Turret/Turret Pose WRT Field", new Pose3d(turretPoseTranslation, turretPoseRotation));

            Logger.recordOutput("Rebuilt/Turret/State/PreviousState", previousState);

            Logger.recordOutput("Rebuilt/Turret/Target/TargetAngleDeg", robotRelativeTargetAngleDeg);
            Logger.recordOutput("Rebuilt/Turret/Target/FieldGoalAngleDeg", fieldRelativeGoalAngle.getDegrees());

            Translation2d currentTargetTranslation = getCurrentTargetPosition();
            if (currentTargetTranslation != null) {
                Logger.recordOutput("Rebuilt/Turret/Target/CurrentTargetTranslation", 
                    new double[] {currentTargetTranslation.getX(), currentTargetTranslation.getY()});
            }

            Pose2d currentTargetPose = new Pose2d(getCurrentTargetPosition(), new Rotation2d(0));
            Logger.recordOutput("Rebuilt/Turret/Target/CurrentTargetPose", currentTargetPose);

            Logger.recordOutput("Rebuilt/Turret/Device/MotorRevolutions", 
                turretMotor.getPosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/AbsoluteEncoderPosition", 
                turretEncoder.getAbsolutePosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/Encoder Relative Position", 
                turretEncoder.getPosition().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorOutputVolts", 
                turretMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorOutputAmps", 
                turretMotor.getTorqueCurrent().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Turret/Device/MotorVelocity (RPS)", 
                cachedMotorVelocity);

            Logger.recordOutput("Rebuilt/Turret/Pos/AtGoal", isAtProfiledGoal());
            Logger.recordOutput("Rebuilt/Turret/Target/TargetOutOfRange", isTargetOutOfRange());
        }
    }
    
    // ========================= Public API =========================
    
    /**
     * Sets the target mode to Hub (for shooting).
     * Target will automatically switch based on alliance.
     */
    public void setTargetHub() {
        currentTargetMode = TargetMode.HUB;
    }
    
    /**
     * Sets the target mode to Ferry (for passing).
     * Target will automatically switch based on alliance.
     */
    public void setTargetFerry() {
        currentTargetMode = TargetMode.FERRY;
    }
    
    /**
     * Sets a custom target position for aiming.
     * Useful for dynamic targets or special game actions.
     * 
     * @param targetPosition Field position (Translation2d) to aim at
     */
    public void setCustomTarget(Translation2d targetPosition) {
        currentTargetMode = TargetMode.CUSTOM;
        customTargetPosition = targetPosition;
    }
    
    public void increaseManualAdjust() {
        manualShift += 5;
    }
    public void decreaseManualAdjust() {
        manualShift -= 5;
    }

    public void resetManualAdjust() {
        manualShift = 0;
    }

    /**
     * Sets the field-relative target for the turret to track.
     * This is a lower-level method for direct angle control.
     * 
     * @param angle Field-relative angle to point at
     * @param velocityRadPerSec Angular velocity of target in field frame (0 for stationary)
     */
    public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
        this.fieldRelativeGoalAngle = angle;
    }
    
    /**
     * Changes the turret's operating state.
     * 
     * @param wantedState Desired state
     */
    public void setState(TurretState wantedState) {
        if (currentState != TurretState.DISABLED) {
            previousState = currentState;
            currentState = wantedState;
            
            // Reset motion profile on state change
            velocityLimiter.reset(Math.toRadians(cachedAngleDeg));
        }
    }

    public Command setStateCommand(TurretState state) {
        return new InstantCommand(() -> setState(state));
    }
    
    /**
     * Convenience method to start aiming at Hub.
     * Equivalent to: setState(AIMING) + setTargetHub()
     */
    public void aimAtHub() {
        setTargetHub();
        setState(TurretState.AIMING);
    }
    
    /**
     * Convenience method to start aiming at Ferry target.
     * Equivalent to: setState(AIMING) + setTargetFerry()
     */
    public void aimAtFerry() {
        setTargetFerry();
        setState(TurretState.AIMING);
    }
    
    /**
     * Convenience method to start aiming at custom position.
     * Equivalent to: setState(AIMING) + setCustomTarget(position)
     * 
     * @param targetPosition Custom field position (Translation2d) to aim at
     */
    public void aimAtCustom(Translation2d targetPosition) {
        setCustomTarget(targetPosition);
        setState(TurretState.AIMING);
    }
    
    /**
     * Zeros the turret encoder at the current position.
     * Call this when the turret is at its mechanical zero position.
     */
    public void zero() {
        turretOffset = -turretEncoder.getAbsolutePosition().getValueAsDouble() * encoderToAngleMultiplier;
    }
    
    /**
     * Resets the Turret's motor internal encoder values to zero.
     */
    public void resetEncoders() {
        turretMotor.setPosition(0);
    }

    /**
     * Syncs the Turret motor's position with the encoders
     */
    public void syncMotorPosToEncoder() {
        turretMotor.setPosition(cachedAngleDeg / TurretConstants.DEGREE_CHANGE_PER_MOTOR_REVOLUTION);
    }

    public void resetEncodersRelativePosition() {
        turretEncoder.setPosition(turretEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    /**
     * Sets the driver controller for rumble feedback.
     * 
     * @param controller Driver's Xbox controller
     */
    public void setDriverController(CommandXboxController controller) {
        this.driverController = controller;
    }
    
    // ========================= Getters =========================
    
    /**
     * Gets the current turret angle in degrees (robot-relative).
     * 
     * @return Current angle in degrees, range [-135, +135]
     */
    public double getCurrentAngleDeg() {
        return cachedAngleDeg;
    }
    
    /**
     * Gets the current turret state.
     * 
     * @return Current operating state
     */
    public TurretState getCurrentState() {
        return currentState;
    }
    
    /**
     * Gets the current target mode.
     * 
     * @return Current targeting mode
     */
    public TargetMode getTargetMode() {
        return currentTargetMode;
    }
    
    /**
     * Gets the field-relative target angle.
     * 
     * @return Target angle in field frame
     */
    public Rotation2d getFieldRelativeTarget() {
        return fieldRelativeGoalAngle;
    }
    
    /**
     * Gets the robot-relative target angle (after wrapping).
     * 
     * @return Target angle in robot frame (degrees)
     */
    public double getRobotRelativeTargetDeg() {
        return robotRelativeTargetAngleDeg;
    }
    
    /**
     * Gets the current target position on the field.
     * 
     * @return Target position, or null if no valid target
     */
    public Translation2d getCurrentTarget() {
        return getCurrentTargetPosition();
    }

    public double getCurrentProfiledSetpoint() {
        return profiledSetpointDeg;
    }
    
        
    public Pose2d getTurretPose() {
        return VisionSubsystem.getInstance().getCurrentBotPoseAllianceUnspecific().transformBy(TurretConstants.OFFSET_WRT_CENTER_OF_ROBOT);
    }
    /**
     * Checks if turret is within tolerance of target.
     * 
     * @return true if within default tolerance (7.5°)
     */
    public boolean isAtProfiledGoal() {
        if (DriverStation.isAutonomous()) {
            return isAtProfiledGoal(12.5);
        }
        return isAtProfiledGoal(7.5);
    }
    
    /**
     * Checks if turret is within specified tolerance of target.
     * 
     * @param toleranceDeg Allowable error in degrees
     * @return true if within tolerance
     */
    public boolean isAtProfiledGoal(double toleranceDeg) {
        if ((Math.abs(Math.abs(profiledSetpointDeg) - MAX_ANGLE_DEG) < 0.5))
            return false;

        double error = Math.abs(cachedAngleDeg - profiledSetpointDeg);
        return error < toleranceDeg;
    }

    public boolean isTurretGoodToFire() {
        if (profiledSetpointDeg == MIN_ANGLE_DEG || profiledSetpointDeg == MAX_ANGLE_DEG) return false;
        return isAtTotalGoal();
    }

    public boolean isAtTotalGoal() {
        if (DriverStation.isAutonomous()) {
            return isAtTotalGoal(25);
        }
        return isAtTotalGoal(17.5);
    }

    public boolean isAtTotalGoal(double tolerance) {
        double error = Math.abs(cachedAngleDeg - (robotRelativeTargetAngleDeg + manualShift));
        return error < tolerance;
    }
    
    /**
     * Gets how far the target is beyond the turret's range.
     * Positive means target is beyond max limit, negative means below min limit.
     * 
     * @return Degrees past available range (0 if within range)
     */
    public double getDegreesPastRange() {
        if (robotRelativeTargetAngleDeg > MAX_ANGLE_DEG) {
            return robotRelativeTargetAngleDeg - MAX_ANGLE_DEG;
        } else if (robotRelativeTargetAngleDeg < MIN_ANGLE_DEG) {
            return robotRelativeTargetAngleDeg - MIN_ANGLE_DEG;
        }
        return 0.0;
    }
    
    // ========================= Singleton =========================
    
    /**
     * Gets the singleton instance of TurretSubsystem.
     * 
     * @return The TurretSubsystem instance
     */
    public static synchronized TurretSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TurretSubsystem();
            INSTANCE.initialize();
        }
        return INSTANCE;
    }

    public void initialize() {
        //Logger.recordOutput("Rebuilt/Turret/Subsystem Initizalized", true);
    }

    public TalonFX getMotor() {
        return turretMotor;
    }
}