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
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.VisionSubsystem.ZoneState;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance;

    // States
    public enum AngleState {
        DISABLED,
        ZEROING,
        AIMING,
        DEADZONE,
        STORING,
        TUNING
    }

    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING,
        HOLD_IN,
    }

    // Other subsystems
    private CoordinationSubsystem coordSub; // = RobotContainer.coordSub;
    private VisionSubsystem visionSub; // = RobotContainer.visionSub;

    private AngleState currentAngleState = AngleState.ZEROING;
    private ShooterState currentShooterState = ShooterState.NOT_SHOOTING;

    private double targetHoodHeight = 0;
    private double targetHoodAngleOverride = 0;
    private double targetShooterVeloc = 0;

    private double hoodAngleMax = 14.3;
    private double hoodAngleMin = 0; //old: 3.5
    private double hoodFerryMotorRev = 14.15;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 8;
    private double distanceAwayToUseLowDegree = 1.2;
    private double distanceAwayToUseHighDegree = 1.78;

    // Motors
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    // Hood Motion Magic parameters
    private LoggedNetworkNumber logHoodMMVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Velocity", 800);
    private LoggedNetworkNumber logHoodMMAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Acceleration", 800);
    private double hoodMMVelocity = logHoodMMVeloc.get();
    private double hoodMMAcceleration = logHoodMMAccel.get();

    private LoggedNetworkNumber logHoodMMKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", .1);
    private LoggedNetworkNumber logHoodMMKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.095);
    private LoggedNetworkNumber logHoodMMKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0);
    private LoggedNetworkNumber logHoodMMKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 15);
    private LoggedNetworkNumber logHoodMMKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
    private LoggedNetworkNumber logHoodMMKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
    private LoggedNetworkNumber logHoodMMKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.3);

    private LoggedNetworkNumber logHoodMotorRevTarget = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Target", 3.5);
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters
    private LoggedNetworkNumber logOuttakeVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0.023549);
    private LoggedNetworkNumber logOuttakeVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.11935);
    private LoggedNetworkNumber logOuttakeVKA = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kA", 0.025517);
    private LoggedNetworkNumber logOuttakeVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", 0.4);
    private LoggedNetworkNumber logOuttakeVKD = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kD", 0.0);
    private LoggedNetworkNumber logOuttakeVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 2.0);
    private LoggedNetworkNumber logOuttakeVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 9450);
    private double outtakeVKFF = logOuttakeVKFF.get();
    private final int outtakeVSlot = 0;
    private final MotionMagicVelocityVoltage outtakeVRequest;

    // Tuning velocity table
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);
    private LoggedNetworkBoolean logIsTuningShooterVelocity = new LoggedNetworkBoolean("/Tuning/Shooter/Outtake/TuningShooterVelocity?", false);
    
    // Flags
    private boolean isTuningShooterVelocity = logIsTuningShooterVelocity.get();
    private boolean isOverrideAngle = false;
    private boolean isZeroingDone = false;
    private boolean hasSwitchedBack = false;
    private boolean hasReachedShooterVelocity = false;
    private boolean swappedMotors = false;

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    private double manualShift = 0; // For manually adjusting shooter velocity on the fly, added to getShooterVeloc()
    private double wheelWareAdjustment = 1.5 - 3.5;

    // Timer
    private Timer switchSlotBackTimer = new Timer();
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
        swappedMotors = false;
        configureAngleMotors();
        configureShooterMotors();
        addData();
        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot).withEnableFOC(RobotConstants.ENABLE_FOC);

        m_sysIdRoutine = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null,        // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
                    (state) -> Logger.recordOutput("Rebuilt/Shooter/Outtake/SysID/State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> shooterLeftMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
                    null,
                    this
                )
            );

    }

    public void addData() {

        shootingDistanceToVelocity.put(6.2, 51.5 + wheelWareAdjustment - .25);
        shootingDistanceToVelocity.put(5.684, 49.75 + wheelWareAdjustment + .5);
        shootingDistanceToVelocity.put(5.095, 46.0 + wheelWareAdjustment + .75);
        shootingDistanceToVelocity.put(4.101, 42.5 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(3.465, 41.0 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(3.0, 38.75 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(2.4, 36.5 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(1.73, 33.55 + wheelWareAdjustment);
        shootingDistanceToVelocity.put(1.1, 30.0 + wheelWareAdjustment);

        shootingDistanceToTime.put(5.095, 1.4525);
        shootingDistanceToTime.put(4.101, 1.374);
        shootingDistanceToTime.put(3.465, 1.3325);
        shootingDistanceToTime.put(3.0, 1.235);
        shootingDistanceToTime.put(2.4, 1.1383);
        shootingDistanceToTime.put(1.73, 1.032);
        shootingDistanceToTime.put(1.1, 0.956);
        


        // Ferrying Data Velocity
        // ferryingDistanceToVelocity.put(4.59, 34.5);
        // ferryingDistanceToVelocity.put(4.962, 35.72);
        // ferryingDistanceToVelocity.put(7.622, 42.03);
        // ferryingDistanceToVelocity.put(11.26, 52.0);
        // ferryingDistanceToVelocity.put(16.26, 62.75);

        ferryingDistanceToVelocity.put(4.59, 34.5 + 2.5);
        ferryingDistanceToVelocity.put(4.962, 35.72 + 2.5);
        ferryingDistanceToVelocity.put(7.622, 42.03 + 8);
        ferryingDistanceToVelocity.put(11.26, 52.0 + 35);
        ferryingDistanceToVelocity.put(14.26, 110.0);

        // Ferrying Data Time
        ferryingDistanceToTime.put(4.59, 1.41);
        ferryingDistanceToTime.put(11.26, 1.96);


    }

    public void configureAngleMotors() {
        angleMotorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE_VALUE;
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;

        Slot0Configs slot0Config = new Slot0Configs().withKS(logHoodMMKS.get())
                                                    .withKV(logHoodMMKV.get())
                                                    .withKA(logHoodMMKA.get())
                                                    .withKP(logHoodMMKP.get())
                                                    .withKI(logHoodMMKI.get())
                                                    .withKD(logHoodMMKD.get())
                                                    .withKG(logHoodMMKG.get());
        angleMotorConfig.Slot0 = slot0Config;
        
        Slot1Configs slot1Config = new Slot1Configs().withKS(logHoodMMKS.get())
                                                    .withKV(logHoodMMKV.get())
                                                    .withKA(logHoodMMKA.get())
                                                    .withKP(logHoodMMKP.get())
                                                    .withKI(logHoodMMKI.get())
                                                    .withKD(logHoodMMKD.get())
                                                    .withKG(logHoodMMKG.get());
        angleMotorConfig.Slot1 = slot1Config;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(logHoodMMAccel.get())
                                                               .withMotionMagicCruiseVelocity(logHoodMMVeloc.get());
        angleMotorConfig.MotionMagic = mmConfigs;

        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodAngleMax; // Hood angle limit
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        angleMotor.getConfigurator().apply(angleMotorConfig);
    }

   private void configureShooterMotors() {
        shooterMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterMotorsConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        shooterMotorsConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLimit = 70;
        shooterMotorsConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;


        Slot0Configs slot0Config = new Slot0Configs().withKS(logOuttakeVKS.get())
                                                    .withKV(logOuttakeVKV.get())
                                                    .withKA(logOuttakeVKA.get())
                                                    .withKP(logOuttakeVKP.get())
                                                    .withKD(logOuttakeVKD.get());

        shooterMotorsConfig.Slot0 = slot0Config;

        shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();

        shooterLeftMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterMotorsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterRightMotor.getConfigurator().apply(shooterMotorsConfig);

        shooterRightMotor.setControl(new Follower(shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean autoZeroing() {
        if(Math.abs(angleMotor.getMotorVoltage().getValueAsDouble()) > 0.4 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) < 0.05) {
            angleMotor.setPosition(0);
            angleMotor.setVoltage(0);
            setState(currentShooterState, AngleState.AIMING);
            coordSub.setState(AbsoluteStates.PREPARING_FOR_SHOOTING, false);
            return true;
        }
        return false;
    }

    public void zeroEncoder() {
        if (DriverStation.isDisabled()) {
            angleMotor.setPosition(0);
        }
    }

    @Override
    public void periodic() {
        // cachedDistanceToTarget = visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget());

        if (!isZeroingDone) {
            currentAngleState = AngleState.ZEROING;
            isZeroingDone = autoZeroing();
        }
        
        // if(!shooterLeftMotor.isConnected() && !swappedMotors) {
        //     swappedMotors = true;
        //     shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
        //     shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
        // }

        updateReachedShooterVelocityLogic();

        publishLogs();

        if(switchSlotBackTimer.get() > 2 && currentAngleState == AngleState.AIMING && !hasSwitchedBack) {
            hasSwitchedBack = true;
            slotNum = 0;
            hoodMMRequest.withVelocity(hoodMMVelocity)
                         .withAcceleration(hoodMMAcceleration);
        }
        applyState();
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Hood/currentState", currentAngleState);
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointMotorRevs", targetHoodHeight);
        // Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointDegrees", targetHoodHeight * degreesPerRev + degreesToZero);
        Logger.recordOutput("Rebuilt/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Hood/Left Supply Amps", angleMotor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Shooter/Hood/current degrees", angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero);
        // Logger.recordOutput("Rebuilt/Shooter/Hood/Is at Goal?", isAngleWithinTolerance());

        Logger.recordOutput("Rebuilt/Shooter/Outtake/currentState", currentShooterState);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/velocitySetpoint", targetShooterVeloc + manualShift);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Left Supply Amps", shooterLeftMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Left current velocity", shooterLeftMotor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Shooter/Outtake/Left motor voltage", shooterLeftMotor.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Shooter/Outtake/Left motor Position", shooterLeftMotor.getPosition().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Shooter/Outtake/Is at Goal?", isVelocityWithinTolerance());

        Logger.recordOutput("Rebuilt/Shooter/Outtake/Manual Adjustment", manualShift);


        if (logDebuggingToggle.get()) { // To get rid of loop overruns

            
            Logger.recordOutput("Rebuilt/Shooter/Time To Shoot", getShootingDistanceToTime(cachedDistanceToTarget));

            // Hood logs
            Logger.recordOutput("Rebuilt/Shooter/Hood/Took target", hoodAngleFunction(cachedDistanceToTarget));
            Logger.recordOutput("Rebuilt/Shooter/Hood/voltage", angleMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Shooter/Hood/Request Accel", hoodMMRequest.Acceleration);
            Logger.recordOutput("Rebuilt/Shooter/Hood/Request Veloc", hoodMMRequest.Velocity);

            // Logger.recordOutput("Rebuilt/Shooter/Hood/Override hood setpoint", targetHoodAngleOverride);
            // Logger.recordOutput("Rebuilt/Shooter/Hood/Overriding hood", isOverrideAngle);

            // Outtake logs
            // Logger.recordOutput("Rebuilt/Shooter/Outtake/Right motor voltage", shooterRightMotor.getMotorVoltage().getValueAsDouble());
            // Logger.recordOutput("Rebuilt/Shooter/Outtake/TargetOverrideVelocity", logTuningShooterVelocity.get());
            // Logger.recordOutput("Rebuilt/Shooter/Outtake/Tuning?", isTuningShooterVelocity);

            // Logger.recordOutput("Rebuilt/Shooter/Outtake/Right Motor Position", shooterRightMotor.getPosition().getValueAsDouble());
        }
    }

    public void applyState() {
        switch (currentShooterState) {
            case SHOOTING:
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(getShooterVeloc()));
                break;

            case NOT_SHOOTING:
                shooterLeftMotor.setVoltage(0);
                break;
            
            case HOLD_IN:
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(-10));
                break;

            default:
                break;
        }
        switch (currentAngleState) {
            case ZEROING:
                // if (TurretSubsystem.getInstance().getCurrentState() == TurretSubsystem.TurretState.INIT && TurretSubsystem.getInstance().isAtProfiledGoal(10)) {
                    // angleMotor.setVoltage(-0.6);
                // } else {
                    angleMotor.setControl(new VoltageOut(-1.0));
                // }
                break;

            case AIMING:
                angleMotor.setControl(hoodMMRequest.withPosition(getShooterAngle()).withSlot(slotNum));
                break;

            case STORING:
                // if (TurretSubsystem.getInstance().getCurrentState() == TurretSubsystem.TurretState.STORING && TurretSubsystem.getInstance().isAtProfiledGoal(15)) {
                    angleMotor.setControl(hoodMMRequest.withPosition(0).withSlot(slotNum));
                // }
                break;

            case TUNING:
                double targetAngle = MathUtil.clamp(logHoodMotorRevTarget.get(), hoodAngleMin, hoodAngleMax);
                angleMotor.setControl(hoodMMRequest.withPosition(targetAngle).withSlot(slotNum));
                break;
                
            default:
                break;
        }
    }


    // --------------------------------------- GETTERS --------------------------------------- //
    public double getShooterVeloc() {
        if(!isTuningShooterVelocity) {
            cachedDistanceToTarget = visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget());
            switch (coordSub.getCurrentZone()) {

                case SHOOTING_ZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case CLIMB_DEADZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case FERRY_ZONE_TOP:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 90; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;
                    
                case FERRY_ZONE_BOTTOM:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 90; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;

                case FERRY_ZONE_MID:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    // if(cachedDistanceToTarget > 14.55) {
                    //     targetShooterVeloc = 60; //TODO: PRACTICE: remove? recalc?
                    // }
                    break;
                    
                case SOFT_TRENCH_DEADZONE:
                    targetShooterVeloc = shootingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case HARD_TRENCH_DEADZONE:
                    targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
                    break;

                case BACK_SHOOT:
                    targetShooterVeloc = 70;
                    break;
                case SHOOT_OVER_NET:
                    targetShooterVeloc = 90;
                    break;

                default:
                    break;
            }
            if(VisionSubsystem.getInstance().getUsingCustomTarget()) {
                targetShooterVeloc = ferryingDistanceToVelocity.get(cachedDistanceToTarget);
            }
        }
        else {
            targetShooterVeloc = logTuningShooterVelocity.get();
        }
        targetShooterVeloc = MathUtil.clamp(targetShooterVeloc, 0, 100);
        return targetShooterVeloc + manualShift;
    }

    public double getShooterAngle() {
        if(!isOverrideAngle) {
            double distance = visionSub.getDistanceToTarget(visionSub.getAdjustedCurrentTarget());
            switch (coordSub.getCurrentZone()) {
                case SHOOTING_ZONE:
                    targetHoodHeight = hoodAngleFunction(distance);
                    break;
                case CLIMB_DEADZONE:
                    targetHoodHeight = hoodAngleFunction(distance);
                    break;
                    
                case FERRY_ZONE_TOP:
                    // targetHoodHeight = hoodAngleFunction(distance);
                    targetHoodHeight = hoodFerryMotorRev;
                    break; 
                    
                case FERRY_ZONE_BOTTOM:
                    // targetHoodHeight = hoodAngleFunction(distance);
                    targetHoodHeight = hoodFerryMotorRev;
                    break;

                case FERRY_ZONE_MID:
                    targetHoodHeight = hoodFerryMotorRev;
                    break;

                case HARD_TRENCH_DEADZONE:
                    setAngleState(AngleState.STORING);
                    break;

                case SOFT_TRENCH_DEADZONE:
                    targetHoodHeight = hoodAngleFunction(distance);
                    break;

                case BACK_SHOOT:
                    targetHoodHeight = MathUtil.clamp(getAsRevs(20), hoodAngleMin, hoodAngleMax);
                    break;

                case SHOOT_OVER_NET:
                    targetHoodHeight = hoodAngleFunctionShootOverNet(distance);
                    break;

                default:
                    return 0;
            }
            if(VisionSubsystem.getInstance().getUsingCustomTarget()) {
                targetHoodHeight = hoodAngleFunction(distance);
            }
        }
        else {
            targetHoodHeight = targetHoodAngleOverride;
        }
        return targetHoodHeight;

    }

    public double getShootingDistanceToTime(double distance) {
        return shootingDistanceToTime.get(distance);
    }

    public double getFerryingDistanceToTime(double distance) {
        return ferryingDistanceToTime.get(distance);
    }

    public double hoodAngleFunction(double distance) {
        // double wantedAngle = (-60/(distance + 0.8)) + 47;
        // double wantedAngle = (-200/(distance + 6)) + 39;
        double wantedAngle = -(200/(distance + 5)) + 47;

        double wantedRevs = getAsRevs(wantedAngle);
        if(distance <= distanceAwayToUseLowDegree) {
            wantedRevs = 0; 
        }
        // if(distance >= distanceAwayToUseHighDegree && coordSub.getCurrentZone() != ZoneState.SHOOTING_ZONE) {
        //     wantedRevs = hoodAngleMax;
        // }
        return MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax);
    }

    public double hoodAngleFunctionShootOverNet(double distance) {
        // double wantedAngle = (-120/(distance + 4.4)) + 45;
        double wantedAngle = (-600/(distance + 28)) + 39;
        double wantedRevs = getAsRevs(wantedAngle);
        if(distance <= distanceAwayToUseLowDegree) {
            wantedRevs = 0; 
        }
        // if(distance >= distanceAwayToUseHighDegree && coordSub.getCurrentZone() != ZoneState.SHOOTING_ZONE) {
        //     wantedRevs = hoodAngleMax;
        // }
        return MathUtil.clamp(wantedRevs, hoodAngleMin, hoodAngleMax);
    }

    public double getHoodAngleDegrees() {
        return angleMotor.getPosition().getValueAsDouble() * degreesPerRev + degreesToZero; // add + 15 if you want true angle
    }

    public double getAsRevs(double degrees) {
        return (degrees-degreesToZero) / degreesPerRev;
    }

    public AngleState getCurrentAngleState() {
        return currentAngleState;
    }

    // Checks
    public boolean isAngleWithinTolerance() {
        return isAngleWithinTolerance(2.75);
    }

    public boolean isAngleWithinTolerance(double tolerance) {
        return Math.abs(targetHoodHeight - angleMotor.getPosition().getValueAsDouble()) < tolerance;
    }

    public boolean isVelocityWithinTolerance() {
        return isVelocityWithinTolerance(16);
    }

    public boolean isVelocityWithinTolerance(double tolerance) {
        double shooterVeloc = shooterLeftMotor.getVelocity().getValueAsDouble();
        if (targetShooterVeloc >= 75) {
            return true;
        }
        return Math.abs(targetShooterVeloc + manualShift - shooterVeloc) < tolerance;
    }

    public boolean isShooterReady() { //TODO: Add back at some point ¯\_(ツ)_/¯
        if(!hasReachedShooterVelocity) {
            return (isAngleWithinTolerance() && isVelocityWithinTolerance());
        }
        // return false;
        return (isAngleWithinTolerance() && isVelocityWithinTolerance(20));
        // return (isAngleWithinTolerance());
        // return true;
    }
    
    private void updateReachedShooterVelocityLogic() {
        if (currentShooterState != ShooterState.SHOOTING) {
            hasReachedShooterVelocity = false;
        } else {
            if (Math.abs(shooterLeftMotor.getVelocity().getValueAsDouble() - getShooterVeloc()) < 3) {
                hasReachedShooterVelocity = true;
            }
        }
    }

    // -------------------------------------- SETTERS -------------------------------------- //
    
    /**
     * Set shooter state
     * @param wantedShooterState
     * @param wantedAngleState
     */
    public void setState(ShooterState wantedShooterState, AngleState wantedAngleState) {
        if(currentAngleState == AngleState.ZEROING || currentAngleState == AngleState.STORING) {
            hoodMMRequest.withVelocity(300)
                         .withAcceleration(300);

            switchSlotBackTimer.restart();
            hasSwitchedBack = false;
        }
        if(wantedAngleState == AngleState.ZEROING) {
            isZeroingDone = false;
        }
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public Command setStateCommand(ShooterState wantedShooterState, AngleState wantedAngleState) {
        return new InstantCommand(() -> setState(wantedShooterState, wantedAngleState));
    }

    public Command setShooterStateCommand(ShooterState wantedShooterState) {
        return setStateCommand(wantedShooterState, currentAngleState);
    }

    public Command setAngleState(AngleState wantedAngleState) {
        return setStateCommand(currentShooterState, wantedAngleState);
    }

    public void setOverrideModeAngle(boolean isOverrideAngle) {
        this.isOverrideAngle = isOverrideAngle;
    }

    public void setOverrideModeShooter() {
        this.isTuningShooterVelocity = true;
    }

    public void setOverrideHoodAngle(double targetHoodAngleOverride) {
        this.targetHoodAngleOverride = MathUtil.clamp(getAsRevs(targetHoodAngleOverride), 0, hoodAngleMax);
    }

    public void addManualVelocityShift() {
        manualShift += 0.5;
    }

    public void addManualVelocityShift(double value) {
        manualShift += value;
    }

    public void subtractManualVelocityShift() {
        manualShift -= 0.5;
    }

    public void resetManualVelocityShift() {
        manualShift = 0;
    }

    // -------------------------------------------- UPDATE BASED ON LOGS -------------------------------------------- //
    public void updateShooterConstants() {
        boolean updateOuttakeVals = false;
        boolean updateAngleVals = false;
        boolean updateSetpoints = false;

        updateSetpoints = (isTuningShooterVelocity != logIsTuningShooterVelocity.get());
        if(updateSetpoints) {
            isTuningShooterVelocity = logIsTuningShooterVelocity.get();
        }

        updateOuttakeVals = (shooterMotorsConfig.Slot0.kS != logOuttakeVKS.get())
                    || (shooterMotorsConfig.Slot0.kV != logOuttakeVKV.get())
                    || (shooterMotorsConfig.Slot0.kA != logOuttakeVKA.get())
                    || (shooterMotorsConfig.Slot0.kP != logOuttakeVKP.get())
                    || (shooterMotorsConfig.Slot0.kD != logOuttakeVKD.get())
                    || (outtakeVKFF != logOuttakeVKFF.get())
                    || (shooterMotorsConfig.MotionMagic.MotionMagicAcceleration != logOuttakeVAccel.get());

        if (updateOuttakeVals) {
            shooterMotorsConfig.Slot0.kS = logOuttakeVKS.get();
            shooterMotorsConfig.Slot0.kV = logOuttakeVKV.get();
            shooterMotorsConfig.Slot0.kA = logOuttakeVKA.get();
            shooterMotorsConfig.Slot0.kP = logOuttakeVKP.get();
            outtakeVKFF = logOuttakeVKFF.get();

            shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();
            
            configureShooterMotors();
        }


        updateAngleVals = (angleMotorConfig.Slot0.kS != logHoodMMKS.get())
                    || (angleMotorConfig.Slot0.kV != logHoodMMKV.get())
                    || (angleMotorConfig.Slot0.kA != logHoodMMKA.get())
                    || (angleMotorConfig.Slot0.kP != logHoodMMKP.get())
                    || (angleMotorConfig.Slot0.kI != logHoodMMKI.get())
                    || (angleMotorConfig.Slot0.kD != logHoodMMKD.get())
                    || (angleMotorConfig.Slot0.kG != logHoodMMKG.get())
                    || (angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logHoodMMVeloc.get())
                    || (angleMotorConfig.MotionMagic.MotionMagicAcceleration != logHoodMMAccel.get());

        if (updateAngleVals) {
            angleMotorConfig.Slot0.kS = logHoodMMKS.get();
            angleMotorConfig.Slot0.kV = logHoodMMKV.get();
            angleMotorConfig.Slot0.kA = logHoodMMKA.get();
            angleMotorConfig.Slot0.kP = logHoodMMKP.get();
            angleMotorConfig.Slot0.kI = logHoodMMKI.get();
            angleMotorConfig.Slot0.kD = logHoodMMKD.get();
            angleMotorConfig.Slot0.kG = logHoodMMKG.get();

            configureAngleMotors();

            hoodMMVelocity = logHoodMMVeloc.get();
            hoodMMAcceleration = logHoodMMAccel.get();

            hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withSlot(slotNum);//.withEnableFOC(RobotConstants.ENABLE_FOC);
        }
        
    }

    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
            instance.initizalize();
            // System.out.println("New shooter instance: " + instance.toString());
        }
        return instance;
    }
    public void initizalize() {
        visionSub = VisionSubsystem.getInstance();
        coordSub = CoordinationSubsystem.getInstance();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(angleMotor);
        arrList.add(shooterLeftMotor);
        arrList.add(shooterRightMotor);
        return arrList;
    }

}
