package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.VisionSubsystem.ZoneState;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    // States
    // Zeroing, Aiming, Deadzone, Storing, Tuning
    public enum AngleState {
        ZEROING,
        AIMING,
        DEADZONE,
        STORING,
        TUNING,
        DISABLED,
    }

    // Shooting, not shooting
    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING,
    }

    private AngleState hoodState = AngleState.ZEROING;
    private ShooterState shooterState = ShooterState.NOT_SHOOTING;
    // Other subsystems

    
    private double angle = 0;
    // ^This is for manual testing
    private double mannualVelocityShift = 0;
    private double hoodAngleMax = 14.3;
    private double hoodAngleMin = 0; //old: 3.5
    private double hoodFerryMotorRev = 14.15;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 8;
    private double distanceAwayToUseLowDegree = 1.2;
    private double distanceAwayToUseHighDegree = 1.78;
    private double targetHood = 0;
    private double targetShooter = 0;

    // Motors
    // 2 Shooter motors, 1 Hood angle motor
    private TalonFX leftShooterMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID,RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX rightShooterMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID,RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID,RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    // Configurators
    TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();

    // Hood Motion Magic parameters
    // DynamicMotionMagic - Allows FOC
    // Veloc, Accel, S, V, A, P, I, D, G
    private DynamicMotionMagicVoltage hoodMMRequest;

    LoggedNetworkNumber hoodMMVelocity = new LoggedNetworkNumber("Tuning/Shooter/Outtake/Hood/Velocity", 10);
    LoggedNetworkNumber hoodMMAcceleration = new LoggedNetworkNumber("Tuning/Shooter/Outtake/Hood/Acceleration",10);
    LoggedNetworkNumber hoodMMS = new LoggedNetworkNumber("Tuning/Shooter/Hood/KS",0);
    LoggedNetworkNumber hoodMMV = new LoggedNetworkNumber("Tuning/Shooter/Hood/KV",0);
    LoggedNetworkNumber hoodMMA = new LoggedNetworkNumber("Tuning/Shooter/Hood/KA",0);
    LoggedNetworkNumber hoodMMP = new LoggedNetworkNumber("Tuning/Shooter/Hood/KP",0);
    LoggedNetworkNumber hoodMMI = new LoggedNetworkNumber("Tuning/Shooter/Hood/KI",0);
    LoggedNetworkNumber hoodMMD = new LoggedNetworkNumber("Tuning/Shooter/Hood/KD",0);
    LoggedNetworkNumber hoodMMG = new LoggedNetworkNumber("Tuning/Shooter/Hood/KG",0);

    // Shooter Outtake Velocity Motion Magic Parameters
    // Velocity Voltage S,V,A,P,D, Accel

    private final MotionMagicVelocityVoltage outtakeVRequest;
    LoggedNetworkNumber outtakeVVelocity = new LoggedNetworkNumber("Tuning/Shooter/Outtake/Hood/Velocity", 10);
    LoggedNetworkNumber outtakeVAcceleration = new LoggedNetworkNumber("Tuning/Shooter/Outtake/Hood/Acceleration",10);
    LoggedNetworkNumber outtakeVS = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KS",0);
    LoggedNetworkNumber outtakeVV = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KV",0);
    LoggedNetworkNumber outtakeVA = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KA",0);
    LoggedNetworkNumber outtakeVP = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KP",0);
    LoggedNetworkNumber outtakeVI = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KI",0);
    LoggedNetworkNumber outtakeVD = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KD",0);
    LoggedNetworkNumber outtakeVG = new LoggedNetworkNumber("Tuning/Shooter/Outtake/KG",0);

    // Tuning velocity table
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);
    private LoggedNetworkBoolean logIsTuningShooterVelocity = new LoggedNetworkBoolean("/Tuning/Shooter/Outtake/TuningShooterVelocity?", false);
    
    // Flags

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    // Timer
    int slotNum = 0;

    // Data tables
    InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shootingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ferryingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap ferryingOverNetDistanceToVelocity = new InterpolatingDoubleTreeMap();

    // Cached Values
    private double cachedDistanceToTarget = 0.0;

    // ----------------------------- CONSTRUCTOR ---------------------------- //
    public ShooterSubsystem() {
        // Set up the requests
        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity.get(), hoodMMAcceleration.get()).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);

    }

    // Data for interpolated data tree for shooter wheel velocity based on distance away from target
    public void addData() {

    }

    public void configureAngleMotors() {
        hoodConfiguration.MotionMagic.MotionMagicCruiseVelocity = outtakeVVelocity.get();
        hoodConfiguration.MotionMagic.MotionMagicAcceleration = outtakeVAcceleration.get();
        hoodConfiguration.Slot0.kS = outtakeVS.get();
        hoodConfiguration.Slot0.kV = outtakeVV.get();
        hoodConfiguration.Slot0.kA = outtakeVA.get();
        hoodConfiguration.Slot0.kP = outtakeVP.get();
        hoodConfiguration.Slot0.kI = outtakeVI.get();
        hoodConfiguration.Slot0.kD = outtakeVD.get();
        hoodConfiguration.Slot0.kG = outtakeVG.get();

        hoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hoodConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        hoodConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
        hoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodAngleMax;

        angleMotor.getConfigurator().apply(hoodConfiguration);
        // Set the neutral mode
        // Set the way the motor spins for pos (find it      phoenix tunner!)
        // Set current limits (stator 40, supply 20)
        // Set motion magic configs
        // Software limits (hood angle max!)
        // Give it to the config
        // If you need help refer to your intake subsystem for syntax!
    }

   private void configureShooterMotors() {
        shooterConfiguration.MotionMagic.MotionMagicCruiseVelocity = hoodMMVelocity.get();
        shooterConfiguration.MotionMagic.MotionMagicAcceleration = hoodMMAcceleration.get();
        shooterConfiguration.Slot0.kS = hoodMMS.get();
        shooterConfiguration.Slot0.kV = hoodMMV.get();
        shooterConfiguration.Slot0.kA = hoodMMA.get();
        shooterConfiguration.Slot0.kP = hoodMMP.get();
        shooterConfiguration.Slot0.kI = hoodMMI.get();
        shooterConfiguration.Slot0.kD = hoodMMD.get();
        shooterConfiguration.Slot0.kG = hoodMMG.get();

        shooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        shooterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfiguration.CurrentLimits.SupplyCurrentLimit = 70;

        rightShooterMotor.getConfigurator().apply(shooterConfiguration);
        leftShooterMotor.getConfigurator().apply(shooterConfiguration);

        leftShooterMotor.setControl(new Follower(rightShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
        // Current limits 80 for stator current limit, 70 for supply current limit
        // Configure both motors!
        // Make one follow the other (just like in the intake!)
    }

    // Zero the hood motor!
    public boolean autoZeroing() {
        return(Math.abs(angleMotor.getMotorVoltage().getValueAsDouble()) > 0.1 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) < 0.1);
    }

    public void zeroEncoder() {
        if (DriverStation.isDisabled()) {
            angleMotor.setPosition(0);
        }
        
    }

    @Override
    public void periodic() {
        // Check zeroing to make sure it stays in zeroing if it's not done
        
        // Apply state
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Angle/HoodPosition",angleMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Angle/HoodTarget",angle);
        Logger.recordOutput("Rebuilt/Shooter/State",hoodState);
        if (logDebuggingToggle.get()) {}
    }

    public void applyState() {
        switch(hoodState) {
            case ZEROING:
                angleMotor.setVoltage(-1);
                if (autoZeroing()) {
                    angleMotor.setPosition(0);
                    hoodState = AngleState.STORING;
                }
                break;
            case AIMING:
                angleMotor.setControl(hoodMMRequest.withPosition(angle).withSlot(0));
                break;
            case DEADZONE:
                break;
            case STORING:
                angleMotor.setControl(hoodMMRequest.withPosition(0).withSlot(0));
                break;
            case TUNING:
                break;
            case DISABLED:
                break;

        }
        switch(shooterState) {
            case SHOOTING:
                break;
            case NOT_SHOOTING:
                rightShooterMotor.setControl(outtakeVRequest.withVelocity(0).withSlot(0));
                break;
        }
        // When shooting, have it's velocity come from the interpolated data tree depending on the distance it's from the target
        
        // Angle states
        // Have the aiming state just be manual control because we dont have limelights
    }
    public void setAngle(double dir) {
        angle = dir;
    }

    // --------------------------------------- GETTERS --------------------------------------- //

    // Switch hood revs to degrees
    public double getHoodAngleDegrees() {
        return angleMotor.getPosition().getValueAsDouble() * degreesPerRev;
    }

    // Switch to revs
    public double getAsRevs(double degrees) {
        return degrees / degreesPerRev;
    }

    // Checks
    // Is velocity in tolerance?
    // Is angle in tolerance?
    // Is the overall shooter ready?
    
    // -------------------------------------- SETTERS -------------------------------------- //
    // Set states
    // -------------------------------------------- UPDATE BASED ON LOGS -------------------------------------------- //
    public void updateShooterConstants() {
        if 
        (
        hoodMMVelocity.get() != hoodConfiguration.MotionMagic.MotionMagicCruiseVelocity ||
        hoodMMAcceleration.get() != hoodConfiguration.MotionMagic.MotionMagicAcceleration ||
        hoodMMS.get() != hoodConfiguration.Slot0.kS ||
        hoodMMV.get() != hoodConfiguration.Slot0.kV ||
        hoodMMA.get() != hoodConfiguration.Slot0.kA ||
        hoodMMP.get() != hoodConfiguration.Slot0.kP ||
        hoodMMI.get() != hoodConfiguration.Slot0.kI ||
        hoodMMD.get() != hoodConfiguration.Slot0.kD ||
        hoodMMG.get() != hoodConfiguration.Slot0.kG
        ) {
            configureAngleMotors();
        }
        if
        (outtakeVVelocity.get() != shooterConfiguration.MotionMagic.MotionMagicCruiseVelocity ||
        outtakeVAcceleration.get() != shooterConfiguration.MotionMagic.MotionMagicAcceleration ||
        outtakeVS.get() != shooterConfiguration.Slot0.kS ||
        outtakeVV.get() != shooterConfiguration.Slot0.kV ||
        outtakeVA.get() != shooterConfiguration.Slot0.kA ||
        outtakeVP.get() != shooterConfiguration.Slot0.kP ||
        outtakeVI.get() != shooterConfiguration.Slot0.kI ||
        outtakeVD.get() != shooterConfiguration.Slot0.kD ||
        outtakeVG.get() != shooterConfiguration.Slot0.kG
        ) {
            configureShooterMotors();
        }
    }
    
    public double getShootingDistanceToTime(double distance) {
        return 0;
    }
    public double getFerryingDistanceToTime(double distance) {
        return 0;
    }
    // Write get instance
    public static ShooterSubsystem getInstance() {
        if (instance == null) instance = new ShooterSubsystem();
        return instance;
    }
    public AngleState getCurrentAngleState() {
        return hoodState;
    }
    public void addManualVelocityShift(double shift) {
        mannualVelocityShift += shift;
    }
    public void addManualVelocityShift() {
        mannualVelocityShift += 0.5;
    }
    public void subtractManualVelocityShift(double shift) {
        mannualVelocityShift -= shift;
    }
    public void subtractManualVelocityShift() {
        mannualVelocityShift -= 0.5;
    }
    public void resetManualVelocityShift() {
        mannualVelocityShift = 0;
    }
    public boolean isShooterReady() {
        return false;
    }
    public void setState(ShooterState Shooter, AngleState Angle) {
        shooterState = Shooter;
        hoodState = Angle;
    }

    public void initizalize() {
    }
    public void manualAngle(double manualAngle) {
        angle = manualAngle;
        hoodState=AngleState.AIMING;
    }
    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(angleMotor);
        arrList.add(leftShooterMotor);
        arrList.add(rightShooterMotor);
        return arrList;
    }

}
