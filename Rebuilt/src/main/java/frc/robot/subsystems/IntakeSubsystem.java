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

    private static volatile IntakeSubsystem INSTANCE;

    //List the states the intake will use
    // Find good value for push intake
    public enum IntakePos {
        INTAKE_OUT,
        INTAKE_IN,
        ZEROING
    }

    public enum IntakeState {
        IDLING,
        SLOW_INTAKING,
        INTAKING,
    }

    //Set the current state of the robot to be idle
    private IntakePos currentPos = IntakePos.ZEROING;
    private IntakeState currentState = IntakeState.IDLING;

    private Timer zeroingTimer = new Timer();
    
    //Set up motors
    TalonFX motorExpander = new TalonFX(IntakeConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    TalonFX motorWheels = new TalonFX(IntakeConstants.DeviceIDs.kWheelMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Intake/Angle/Velocity", 30); //95
    private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("/Tuning/Intake/Angle/Acceleration", 45); //120

    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Intake/Angle/kS", 0.275); // .145
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Intake/Angle/kV", 0.0); // .2
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Intake/Angle/kA", 0); // 0
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Intake/Angle/kP", 100); // 48
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Intake/Angle/kI", 0); // 0
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Intake/Angle/kD", 3.5); // 0.1
    private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Intake/Angle/kG", 0.0); // 0

    private LoggedNetworkNumber logIntakeTarget = new LoggedNetworkNumber("/Tuning/Intake/Angle/Intaking Target", 0.052);
    private LoggedNetworkNumber logIntakingTarget = new LoggedNetworkNumber("/Tuning/Intake/Wheels/Target", 5);

    private double intakingTargetVolts = logIntakingTarget.get();
    
    private double intakeInRevolutions = 0;
    private double intakeOutRevolutions = 0;

    private double intakeSetpoint = 0;

    //Motion Magic Set Up(Intake arm)
    private MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(RobotConstants.ENABLE_FOC);
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private boolean isZeroingDone = false;

    private boolean reachedSetpoint = false;
    private boolean jammed = false;
    private double jamCount = 0;

    private boolean reversing = false;
    private boolean fastIntaking = false;

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotionMagic.MotionMagicAcceleration = logMMAccel.get();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = logMMVeloc.get();

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;

        //Magic Motion
        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kS = logMMKS.get();
        slot0Configs.kV = logMMKV.get();
        slot0Configs.kA = logMMKA.get();
        slot0Configs.kP = logMMKP.get();
        slot0Configs.kI = logMMKI.get();
        slot0Configs.kD = logMMKD.get();
        slot0Configs.kG = logMMKG.get();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        // var slot1Configs = motorConfig.Slot1;
        // slot1Configs.kS = logMMKS.get();
        // slot1Configs.kV = logMMKV.get();
        // slot1Configs.kA = logMMKA.get();
        // slot1Configs.kP = logMMKP.get();
        // slot1Configs.kI = logMMKI.get();
        // slot1Configs.kD = logMMKD.get();
        // slot1Configs.kG = logMMKG.get();
        // slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;

        motorExpander.getConfigurator().apply(motorConfig);

        //Configure intake motors
        TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
        IntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = 80; //60
        IntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        
        motorWheels.getConfigurator().apply(IntakeConfig);
        // motorWheelsFollower.stopMotor();
    }

    public boolean isZeroingDone() {
        if(zeroingTimer.get() > 0.5 && Math.abs(motorExpander.getVelocity().getValueAsDouble()) < .05 && Math.abs(motorExpander.getMotorVoltage().getValueAsDouble()) > 0.2) {
            motorExpander.setPosition(0);
            motorExpander.stopMotor();
            currentPos = IntakePos.INTAKE_IN;
            zeroingTimer.restart();
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if(!isZeroingDone) {
            currentPos = IntakePos.ZEROING;
            isZeroingDone = isZeroingDone();
        }
        applyState();
        publishLogs();
    }

    private void applyState() {
        switch (currentPos) {
            case ZEROING:
                motorExpander.setVoltage(-2.5);

                break;
            case INTAKE_IN:
                motorExpander.setControl(request.withPosition(intakeInRevolutions).withSlot(0));
                intakeSetpoint = intakeInRevolutions;
                break;

            case INTAKE_OUT:
                motorExpander.setControl(request.withPosition(intakeOutRevolutions).withSlot(0));
                intakeSetpoint = intakeOutRevolutions;
                break;

            default:
                motorExpander.setControl(request.withPosition(intakeInRevolutions).withSlot(0));
                intakeSetpoint = intakeInRevolutions;
                break;
        }

        switch (currentState) {
            case IDLING:
                motorWheels.setControl(voltageRequest.withOutput(0));
                break;

            case SLOW_INTAKING:
                motorWheels.setControl(voltageRequest.withOutput(intakingTargetVolts * 0.833333)); //0.58333
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

    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Intake/Angle/Current Pos", currentPos);

        Logger.recordOutput("Rebuilt/Intake/Angle/Position", motorExpander.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Target", currentPos == IntakePos.INTAKE_OUT ? logIntakeTarget.get() : intakeInRevolutions);
        // Logger.recordOutput("Rebuilt/Intake/Angle/Motor Voltage", motorArm.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Intake/Angle/Motor Velocity", motorArm.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Is Zeroing", isZeroingDone);
        // Logger.recordOutput("Rebuilt/Intake/Angle/Zeroing Timer", zeroingTimer.get());
        Logger.recordOutput("Rebuilt/Intake/Angle/Supply Amp Pull", motorExpander.getSupplyCurrent().getValueAsDouble());

        // Logger.recordOutput("Rebuilt/Intake/Wheels/Lead Motor Veloc", motorWheels.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Lead Motor Voltage", motorWheels.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Wheels/Lead Motor Supply Amp Pull", motorWheels.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Intake/Wheels/Follower Motor Veloc", motorWheelsFollower.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Intake/Angle/Temp", motorExpander.getDeviceTemp().getValueAsDouble());
        
        Logger.recordOutput("Rebuilt/Intake/Intake Jammed", jammed);
    }

    public void checkJammed() {
        if(!reachedSetpoint && Math.abs(intakeSetpoint - motorExpander.getPosition().getValueAsDouble()) < .1) {
            reachedSetpoint = true;
        }
        if(currentPos == IntakePos.INTAKE_OUT && reachedSetpoint) {
            if(Math.abs(intakeSetpoint - motorExpander.getPosition().getValueAsDouble()) > 2) {
                jamCount++;
            }
            else {
                jammed = false;
                jamCount = 0;
            }
            if(jamCount > 5) {
                jammed = true;
            }
        }
        else if(!reachedSetpoint && Math.abs(motorExpander.getVelocity().getValueAsDouble()) < .05 && Math.abs(motorExpander.getMotorVoltage().getValueAsDouble()) > 3) {
            jammed = true;
        }
        else {
            jammed = false;
        }
    }

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
            zeroingTimer.restart();
        }
        reachedSetpoint = false;
        currentPos = pos;
    }

    public void setState(IntakeState state) {
        currentState = state;
    }

    public Command setPosAndStateCommand(IntakePos pos, IntakeState state) {
        return new InstantCommand(() -> {setPos(pos); setState(state);});
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakeSubsystem();
        return INSTANCE;
    }

    public void updateConstants() {
        boolean gainsChanged = false;

        if (motorConfig.MotionMagic.MotionMagicAcceleration != logMMAccel.get() ||
            motorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVeloc.get() ||
            motorConfig.Slot0.kS != logMMKS.get() ||
            motorConfig.Slot0.kV != logMMKV.get() ||
            motorConfig.Slot0.kA != logMMKA.get() ||
            motorConfig.Slot0.kP != logMMKP.get() ||
            motorConfig.Slot0.kI != logMMKI.get() ||
            motorConfig.Slot0.kD != logMMKD.get() ||
            motorConfig.Slot0.kG != logMMKG.get()
            ) {
            
            gainsChanged = true;
        }

        if (gainsChanged) {
            configureMotors();
        }

        intakeOutRevolutions = logIntakeTarget.get();
        intakingTargetVolts = logIntakingTarget.get();
    }

    public void init() {
        zeroingTimer.restart();
    }
    public void zeroEncoders() {
        if (DriverStation.isDisabled()) {
            motorExpander.setPosition(0);
        }
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(motorExpander);
        arrList.add(motorWheels);
        return arrList;
    }
}