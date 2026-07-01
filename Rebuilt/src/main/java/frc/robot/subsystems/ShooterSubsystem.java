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
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    private static ShooterSubsystem INSTANCE;

    // States
    // Zeroing, Aiming, Deadzone, Storing, Tuning
    public enum AngleState {
        ZEROING,
        AIMING,
        STORING,
        TUNING,
    }

    // Shooting, not shooting
    public enum ShooterState {
        SHOOTING,
        NOT_SHOOTING,
    }
    
    private ShooterState currentShooterState = ShooterState.NOT_SHOOTING;
    private AngleState currentAngleState = AngleState.STORING;

    // Other subsystems
    
    public final double hoodAngleMax = 14.3;
    public final double hoodAngleMin = 0; //old: 3.5
    private final double hoodFerryMotorRev = 14.15;
    private final double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private final double degreesToZero = 8;
    private final double distanceAwayToUseLowDegree = 1.2;
    private final double distanceAwayToUseHighDegree = 1.78;

    // Motors
    // 2 Shooter motors, 1 Hood angle motor
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    // Configurators
    private TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

    // Hood Motion Magic parameters
    // DynamicMotionMagic - Allows FOC
    // Veloc, Accel, S, V, A, P, I, D, G
    private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/Velocity", 5);
    private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/Acceleration", 5);
    private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kS", 0);
    private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kV", 0);
    private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kA", 0);
    private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kP", 5);
    private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kI", 0);
    private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kD", 0);
    private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("Rebuilt/Shooter/Hood/kG", 0);
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters
    // Velocity Voltage S,V,A,P,D, Accel
    private LoggedNetworkNumber logVVAccel = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/Acceleration", 5);
    private LoggedNetworkNumber logVVKS = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/kS", 0);
    private LoggedNetworkNumber logVVKV = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/kV", 0);
    private LoggedNetworkNumber logVVKA = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/kA", 0);
    private LoggedNetworkNumber logVVKP = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/kP", 5);
    private LoggedNetworkNumber logVVKD = new LoggedNetworkNumber("Rebuilt/Shooter/Outtake/kD", 0);
    private final MotionMagicVelocityVoltage outtakeVRequest;

    // Tuning velocity table
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);
    private LoggedNetworkBoolean logIsTuningShooterVelocity = new LoggedNetworkBoolean("/Tuning/Shooter/Outtake/TuningShooterVelocity?", false);
    
    // Flags
    private boolean isZeroingDone = false;
    private double angleSetpoint = 0.0;
    private double velocSetpoint = 0.0;
    private double manualVelocShift = 0.0;
    private boolean velocReachedSetpoint = false;
    private double angleSetpointTuning = 0.0;
    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    // Data tables
    private InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shootingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    private InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap ferryingDistanceToVelocity = new InterpolatingDoubleTreeMap();

    private InterpolatingDoubleTreeMap ferryingOverNetDistanceToVelocity = new InterpolatingDoubleTreeMap();

    // Cached Values
    private double cachedDistanceToTarget = 0.0;

    // ----------------------------- CONSTRUCTOR ---------------------------- //
    public ShooterSubsystem() {
        // Set up the requests
        hoodMMRequest = new DynamicMotionMagicVoltage(0, logMMVeloc.get(), logMMAccel.get()).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(0);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(RobotConstants.ENABLE_FOC);

        configureAngleMotors();
        configureShooterMotors();
    }

    // Data for interpolated data tree for shooter wheel velocity based on distance away from target
    public void addData() {
    }

    public void configureAngleMotors() {
        // Set the neutral mode
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Set the way the motor spins for pos (find it through phoenix tunner!)
        angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Set current limits
        angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        // Set motion magic configs
        Slot0Configs slot0Configs = angleMotorConfig.Slot0;
        slot0Configs.kS = logMMKS.get();
        slot0Configs.kV = logMMKV.get();
        slot0Configs.kA = logMMKA.get();
        slot0Configs.kP = logMMKP.get();
        slot0Configs.kI = logMMKI.get();
        slot0Configs.kD = logMMKD.get();
        slot0Configs.kG = logMMKG.get();
        // Software limits (hood angle max!)
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodAngleMax;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = hoodAngleMin;
        // Give it to the config
        angleMotor.getConfigurator().apply(angleMotorConfig);
        // If you need help refer to your intake subsystem for syntax!
    }

   private void configureShooterMotors() {
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // Current limits 80 for stator current limit, 70 for supply current limit
        shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;
        // Configure both motors!
        shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
        shooterRightMotor.getConfigurator().apply(shooterMotorConfig);
        // Make one follow the other (just like in the intake!)
        shooterRightMotor.setControl(new Follower(shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // Zero the hood motor!
    public boolean autoZeroing() {
        if (angleMotor.getMotorVoltage().getValueAsDouble() > 0 && Math.abs(angleMotor.getVelocity().getValueAsDouble()) <= 0.1) {
            angleMotor.stopMotor();
            angleMotor.setPosition(0);

            currentAngleState = AngleState.STORING;
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
        // Check zeroing to make sure it stays in zeroing if it's not done
        if (!isZeroingDone) {
            currentAngleState = AngleState.ZEROING;
            isZeroingDone = autoZeroing();
        }
        if (!velocReachedSetpoint) velocReachedSetpoint = Math.abs((velocSetpoint + manualVelocShift) - shooterLeftMotor.getVelocity().getValueAsDouble()) < 0.02;

        // Apply state
        applyState();
        publishLogs();
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Hood/Current State", currentAngleState);
        Logger.recordOutput("Rebuilt/Shooter/Hood/Current Setpoint", angleSetpoint);
        Logger.recordOutput("Rebuilt/Shooter/Hood/Current Angle", angleMotor.getPosition().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Shooter/Outtake/Current State", currentShooterState);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Current Setpoint", velocSetpoint);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Reached Setpoint", velocReachedSetpoint);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Current Velocity", shooterLeftMotor.getVelocity().getValueAsDouble());

        if (logDebuggingToggle.get()) { // To get rid of loop overruns
            Logger.recordOutput("Rebuilt/Shooter/Cached Distance to Target", cachedDistanceToTarget);
            Logger.recordOutput("Rebuilt/Shooter/Zeroing Done", isZeroingDone);

            Logger.recordOutput("Rebuilt/Shooter/Manual Velocity Shift", manualVelocShift);
            Logger.recordOutput("Rebuilt/Shooter/Tuning Angle Setpoint", angleSetpointTuning);
        }
    }

    public void applyState() {
        // When shooting, have it's velocity come from the interpolated data tree depending on the distance it's from the target
        
        // Angle states
        // Have the aiming state just be manual control because we dont have limelights
        //  Actually now we might be getting limelights!
        switch (currentAngleState) {
            case AIMING:
                angleSetpoint = 13.0;
                angleMotor.setControl(hoodMMRequest.withPosition(angleSetpoint));
                break;
            case STORING:
                angleSetpoint = 0;
                angleMotor.setControl(hoodMMRequest.withPosition(angleSetpoint));
                break;
            case TUNING:
                angleSetpoint = angleSetpointTuning;
                angleMotor.setControl(hoodMMRequest.withPosition(angleSetpoint));
                break;
            case ZEROING:
                angleSetpoint = 0.0;
                angleMotor.setControl(new VoltageOut(-1));
                break;
        }

        switch (currentShooterState) {
            case NOT_SHOOTING:
                velocSetpoint = 0.0;
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(velocSetpoint + manualVelocShift));
                break;
            case SHOOTING:
                velocSetpoint = 30.0; // No data yet, so this can't use an interpolated data tree
                shooterLeftMotor.setControl(outtakeVRequest.withVelocity(velocSetpoint + manualVelocShift));
                break;
        }
    }


    // --------------------------------------- GETTERS --------------------------------------- //
    public ShooterState getCurrentShooterState () {
        return currentShooterState;
    }
    public AngleState getCurrentAngleState () {
        return currentAngleState;
    }

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
    public boolean isVelocityInTolerance() {
        return velocReachedSetpoint;
    }
    // Is angle in tolerance?
    public boolean isAngleInTolerance() {
        return Math.abs(angleSetpoint - angleMotor.getPosition().getValueAsDouble()) < 0.02;
    }
    // Is the overall shooter ready?
    public boolean isShooterReady() {
        return isVelocityInTolerance() && isAngleInTolerance();
    }

    public double getShootingDistanceToTime(double distance) { // Currently not implemented due to not having data
        return 0.0;
    }
    public double getFerryingDistanceToTime(double distance) { // Currently not implemented due to not having data
        return 0.0;
    }
    
    public double getManualVelocityShift() {
        return manualVelocShift;
    }
    public double getTuningAngleSetpoint() {
        return angleSetpointTuning;
    }
    
    // -------------------------------------- SETTERS -------------------------------------- //
    // Set states
    public void setState(ShooterState shooterState, AngleState angleState) {
        if (shooterState != currentShooterState) velocReachedSetpoint = false;

        currentShooterState = shooterState;
        currentAngleState = angleState;
    }

    public InstantCommand setStateCommand(ShooterState shooterState, AngleState angleState) {
        return new InstantCommand(() -> setState(shooterState, angleState));
    }

    public void addManualVelocityShift(double amount) {
        manualVelocShift += amount;
    }
    public void addManualVelocityShift() {
        manualVelocShift += 0.5;
    }
    public void subtractManualVelocityShift(double amount) {
        manualVelocShift -= amount;
    }
    public void subtractManualVelocityShift() {
        manualVelocShift -= 0.5;
    }
    public void resetManualVelocityShift() {
        manualVelocShift = 0.0;
    }

    public void setTuningAngleSetpoint(double setpoint) {
        if (currentAngleState == AngleState.TUNING) angleSetpointTuning = setpoint;
    }

    // -------------------------------------------- UPDATE BASED ON LOGS -------------------------------------------- //
    public void updateShooterConstants() {
        // Check if you need to update hood motor motion magic values, then update it
        if (
            angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity != logMMVeloc.get() ||
            angleMotorConfig.MotionMagic.MotionMagicAcceleration != logMMAccel.get() ||
            angleMotorConfig.Slot0.kS != logMMKS.get() ||
            angleMotorConfig.Slot0.kV != logMMKV.get() ||
            angleMotorConfig.Slot0.kA != logMMKA.get() ||
            angleMotorConfig.Slot0.kP != logMMKP.get() ||
            angleMotorConfig.Slot0.kI != logMMKI.get() ||
            angleMotorConfig.Slot0.kD != logMMKD.get() ||
            angleMotorConfig.Slot0.kG != logMMKG.get()
        ) configureAngleMotors();
        // Check if you need to update shooter motors, then update them
        if (
            shooterMotorConfig.MotionMagic.MotionMagicAcceleration != logVVAccel.get() ||
            shooterMotorConfig.Slot0.kS != logVVKS.get() ||
            shooterMotorConfig.Slot0.kV != logVVKV.get() ||
            shooterMotorConfig.Slot0.kA != logVVKA.get() ||
            shooterMotorConfig.Slot0.kP != logVVKP.get() ||
            shooterMotorConfig.Slot0.kD != logVVKD.get()
        ) configureShooterMotors();
    }

    // Write get instance
    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        INSTANCE.initizalize();
        return INSTANCE;
    }

    // Set up the other subsystems
    public void initizalize() {
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(angleMotor);
        arrList.add(shooterLeftMotor);
        arrList.add(shooterRightMotor);
        return arrList;
    }
}
