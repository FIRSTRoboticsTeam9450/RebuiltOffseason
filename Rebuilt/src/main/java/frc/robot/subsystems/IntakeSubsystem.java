package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class IntakeSubsystem extends SubsystemBase{

    // The instance! Should be in every subsystem to make sure there's only one of each
    private static volatile IntakeSubsystem INSTACNE;

    //List the positions the intake will use
    // These include, intaking, storing, trench_store (allows the intake to not hit the trench)
    public enum IntakePos {

    }

    // Different intaking states
    // Not intaking, intaking, slow intake
    public enum IntakeState {
        
    }

    //Set the current state of the robot to be idle

    //Set up the arm motor, encoder and wheel motors
    // TalonFX motor = new TalonFX(motorID, canBUS)
    // Ex: TalonFX motorArm = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    // CANcoder encoder = new CANcoder(canID, canBUS)
    // Make sure you use constants!
    // The wheels use two motors, one leader, one follower
    /* Put your code here */

    // Motor configurater!
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Create tunable logs for each PID parameter, Veloc, Accel, KS, V, A, P, I, D
    // EX: private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", default_value); 
    // /Tuning provides the path for it to be a tunable value
    /* Put your code here */

    private LoggedNetworkNumber logIntakeTarget = new LoggedNetworkNumber("/Tuning/Intake/Angle/Intaking Target", 0.052);
    private LoggedNetworkNumber logIntakingTarget = new LoggedNetworkNumber("/Tuning/Intake/Wheels/Target", 5);

    private double intakingTargetVolts = logIntakingTarget.get();
    private double intakeRevolutions = logIntakeTarget.get();

    // Positions
    private double storeRevolutions = 0.385;
    private double trenchStoreRevolutions = 0.21527;

    // Current setpoint
    private double intakeSetpoint = 0;

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    // Flags

    // Constructor
    public IntakeSubsystem() {
        // Configs

        configureEncoder();
        configureMotors();
    }

    private void configureEncoder() {
        // Call the configurator
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Which way when turned does it go positive
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Wrap point
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;

        // For zeroing (Can get this through phoenix tuner)
        config.MagnetSensor.MagnetOffset = IntakeConstants.kAngleEncoderMagnetOffset;

        // Apply it
        encoderArm.getConfigurator().apply(config);
    }

    private void configureMotors() {
        // Sets coast or brake
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // To limit amp pulls
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        // Set up Magic Motion
        motorConfig.MotionMagic.MotionMagicAcceleration = logMMAccel.get();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = logMMVeloc.get();

        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kS = logMMKS.get();
        slot0Configs.kV = logMMKV.get();
        slot0Configs.kA = logMMKA.get();
        slot0Configs.kP = logMMKP.get();
        slot0Configs.kI = logMMKI.get();
        slot0Configs.kD = logMMKD.get();
        slot0Configs.kG = logMMKG.get();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        // Connect the motor to the encoder
        motorConfig.Feedback.FeedbackRemoteSensorID = encoderArm.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Gear ratio
        motorConfig.Feedback.RotorToSensorRatio = IntakeConstants.kAngleRotorToSensorRatio;
        motorConfig.Feedback.FeedbackRotorOffset = 0;

        motorArm.getConfigurator().apply(motorConfig);

        //Configure intake wheels motors
        TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
        IntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = 80; //60
        IntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        

        
        motorWheels.getConfigurator().apply(IntakeConfig);
        motorWheelsFollower.getConfigurator().apply(IntakeConfig);

        motorWheelsFollower.setControl(new StrictFollower(motorWheels.getDeviceID()));
        // motorWheelsFollower.stopMotor();
    }

    // Used for relative encoders
    public boolean isZeroingDone() {
        // If the motor is not moving but voltage is getting sent to it that means it's zeroed
            // Zero the motor with setPosition()
            // Get out of the zeroing state
            // Set the motor voltage to 0
        return false;
    }


    // Runs every 20 millisecs
    @Override
    public void periodic() {

        applyState();
        publishLogs();
    }

    private void applyState() {
        // Make a switch case for every position state with position request
        // EX: switch (currentPos) {
            // case INTAKING:
            //     motorArm.setControl(request.withPosition(intakeRevolutions).withSlot(0));
            //     intakeSetpoint = intakeRevolutions;
            //     break;
        /* Your code here */

        // Do the same for intaking states with voltageRequest
        // How to send a request for a motor to do something!
        // motor.setControl(___request.with)
        // EX: motorWheels.setControl(voltageRequest.withOutput(0));
        /* Your code here */

    }

    // Log info so you can debug here
    private void publishLogs() {
        // I recommend logging position, target, and states 
        // The path is "Rebuilt/Intake", each / creates a new subsection so it allows for better organization
        // EX: Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);
    }

    /* CREATE SETTERS AND GETTERS */
    
    // You can return an InstantCommand(() -> _____) so you won't have to in RobotContainer()

    // Every subsystem should have one! When using the subsystem in other classes, use getInstance()
    public static synchronized IntakeSubsystem getInstance() {
        if (INSTACNE == null) INSTACNE = new IntakeSubsystem();
        return INSTACNE;
    }

    // To have the tuning system actually update
    // Make sure to call this in Robot.java in disabled periodic
    public void updateConstants() {

        // Check if anything values have changed
            // If something has changed, reconfigure motors so the values update
        /* YOUR CODE HERE */

        intakeRevolutions = logIntakeTarget.get();
        intakingTargetVolts = logIntakingTarget.get();
    }

    public void init() {
        zeroingTimer.restart();
    }

    public void zeroEncoders() {
        if (DriverStation.isDisabled()) {
            motorArm.setPosition(0);
        }
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(motorArm);
        arrList.add(motorWheels);
        arrList.add(motorWheelsFollower);
        return arrList;
    }
}