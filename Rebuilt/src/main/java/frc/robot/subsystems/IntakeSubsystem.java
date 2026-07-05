package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
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
    private static volatile IntakeSubsystem INSTANCE;

    //List the positions the intake will use
    // These include, intaking, storing, trench_store (allows the intake to not hit the trench)
    public enum IntakePos {
        INTAKING,
        STORE,
        TRENCH_STORE,
        DEFENSE,
        PUSH_BAR,
        CLIMB_STORE,
        ZEROING
    }

    // Different intaking states
    // Not intaking, intaking, slow intake
    public enum IntakeState {
        IDLING,
        SLOW_INTAKE,
        INTAKING
    }

    private Timer zeroingTimer;

    //Set the current state of the robot to be idle
    private IntakePos currentPos = IntakePos.STORE;
    private IntakeState currentState = IntakeState.IDLING;
    
    //Set up the arm motor, encoder and wheel motors
    // TalonFX motor = new TalonFX(motorID, canBUS)
    // Ex: TalonFX motorArm = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    // CANcoder encoder = new CANcoder(canID, canBUS)
    // Make sure you use constants!
    // The wheels use two motors, one leader, one follower
    /* Put your code here */

    TalonFX motorArm = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    CANcoder encoderArm = new CANcoder(IntakeConstants.DeviceIDs.kAngleEncoderID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    TalonFX motorWheels = new TalonFX(IntakeConstants.DeviceIDs.kWheelMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    TalonFX motorWheelsFollower = new TalonFX(IntakeConstants.DeviceIDs.kWheelMotorIDFollower, RobotConstants.MAIN_SYSTEMS_CANBUS);

    // Motor configurater!
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Create tunable logs for each PID parameter, Veloc, Accel, KS, V, A, P, I, D
    // EX: private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", default_value); 
    // /Tuning provides the path for it to be a tunable value
    /* Put your code here */
    private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", 0);
    private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("/Tuning/Intake/Angle/Acceleration", 0);

    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Intake/KS", 0);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Intake/KA", 0);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Intake/KV", 0);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Intake/KP", 0);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Intake/KI", 0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Intake/KD", 0);
    private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Intake/KG", 0);

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
    private boolean isZeroingDone = false;
    private boolean jammed = false;
    private boolean reversing = false;

    //sensitivity thresholds
    private double velSensitivity = 0.05;
    private double volSensitivity = 0.15;

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
        if (zeroingTimer.get() < 0.5) return false;
        if (Math.abs(motorArm.getVelocity().getValueAsDouble()) < velSensitivity && Math.abs(motorArm.getMotorVoltage().getValueAsDouble()) > volSensitivity) {
            motorArm.setPosition(0);
            currentPos = IntakePos.STORE;
            return true;
        }
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

        switch (currentPos) {
            case INTAKING:
                motorArm.setControl(request.withPosition(intakeRevolutions).withSlot(0));
                intakeSetpoint = intakeRevolutions;
                break;
            case STORE:
                motorArm.setControl(request.withPosition(storeRevolutions).withSlot(0));
                intakeSetpoint = storeRevolutions;
                break;
            case TRENCH_STORE:
                motorArm.setControl(request.withPosition(trenchStoreRevolutions).withSlot(0));
                intakeSetpoint = trenchStoreRevolutions;
                break;
            case DEFENSE:
                //raise arm up high to avoid collision
                break;
            case PUSH_BAR:
                //arm down
                break;
            case CLIMB_STORE:
                //raise arm high enough for climb
                break;
            
            default:
                //just store i guess
                motorArm.setControl(request.withPosition(storeRevolutions).withSlot(0));
                intakeSetpoint = storeRevolutions;
                break;
        }

        switch (currentState) {
            case IDLING:
                motorWheels.setControl(voltageRequest.withOutput(0));
                break;
            case SLOW_INTAKE:
                motorWheels.setControl(voltageRequest.withOutput(intakingTargetVolts * 0.8));
                break;
            case INTAKING:
                if(reversing) {
                    intakingTargetVolts = -intakingTargetVolts;
                }
                if(CoordinationSubsystem.getInstance().isFastTransfer()) {
                    motorWheels.setControl(voltageRequest.withOutput(10));
                }
                else {
                    motorWheels.setControl(voltageRequest.withOutput(intakingTargetVolts));

                }
                break;
        }

    }

    // public void jamTest() {

        // use existing jam testing logic since i do not have the requirements for the jam test, also jam isnt ever even checked anyway

        // if(!reachedSetpoint && Math.abs(intakeSetpoint - motorArm.getPosition().getValueAsDouble()) < .1) {
        //     reachedSetpoint = true;
        // }
        // if(currentPos == IntakePos.INTAKING && reachedSetpoint) {
        //     if(Math.abs(intakeSetpoint - motorArm.getPosition().getValueAsDouble()) > 2) {
        //         jamCount++;
        //     }
        //     else {
        //         jammed = false;
        //         jamCount = 0;
        //     }
        //     if(jamCount > 5) {
        //         jammed = true;
        //     }
        // }
        // else if(!reachedSetpoint && Math.abs(motorArm.getVelocity().getValueAsDouble()) < .05 && Math.abs(motorArm.getMotorVoltage().getValueAsDouble()) > 3) {
        //     jammed = true;
        // }
        // else {
        //     jammed = false;
        // }

    // }

    // Log info so you can debug here
    private void publishLogs() {
        // I recommend logging position, target, and states 
        // The path is "Rebuilt/Intake", each / creates a new subsection so it allows for better organization
        // EX: Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);

        Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);    
        Logger.recordOutput("Rebuilt/Intake/Angle/Position", motorArm.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Target", currentPos == IntakePos.INTAKING ? logIntakeTarget.get() : storeRevolutions);
        Logger.recordOutput("Rebuilt/Intake/Angle/Is Zeroing", isZeroingDone);
        Logger.recordOutput("Rebuilt/Intake/Angle/Supply Amp Pull", motorArm.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Encoder Absolute Pos", encoderArm.getAbsolutePosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Lead Motor Voltage", motorWheels.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Lead Motor Supply Amp Pull", motorWheels.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Follower Motor Voltage", motorWheelsFollower.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Follower Motor Supply Amp Pull", motorWheelsFollower.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Temp", motorArm.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Intake Jammed", jammed);

    }

    /* CREATE SETTERS AND GETTERS */
    public boolean getJammed() {
        return jammed;
    }
    public IntakePos getCurrentIntakePos() {
        return currentPos;
    }

    public void setPos(IntakePos pos) {
        if (currentPos == IntakePos.ZEROING) return;
        if (pos == IntakePos.ZEROING) {
            isZeroingDone = false;
            zeroingTimer.reset();
        }
        currentPos = pos;
    }

    public void setState(IntakeState state) {
        currentState = state;
    }

    // You can return an InstantCommand(() -> _____) so you won't have to in RobotContainer()

    // Every subsystem should have one! When using the subsystem in other classes, use getInstance()
    public static synchronized IntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakeSubsystem();
        return INSTANCE;
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