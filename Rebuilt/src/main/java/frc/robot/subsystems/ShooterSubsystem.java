package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;

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

    private AngleState currentAngleState = AngleState.ZEROING;
    private ShooterState currentShooterState = ShooterState.NOT_SHOOTING;

    private double targetHoodHeight = 0;
    private double targetShooterVeloc = 0;

    private double hoodAngleMax = 14.3;
    private double hoodAngleMin = 0; //old: 3.5
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 8;

    // Motors
    private TalonFX angleMotor = new TalonFX(ShooterConstants.DeviceIDs.kAngleMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterLeftMotor = new TalonFX(ShooterConstants.DeviceIDs.kLeftShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private TalonFX shooterRightMotor = new TalonFX(ShooterConstants.DeviceIDs.kRightShooterMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();

    // Hood Motion Magic parameters (all tunable live from AdvantageScope)
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

    // Manual hood setpoint (motor revs) - tweak this to control the hood angle
    private LoggedNetworkNumber logHoodMotorRevTarget = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Target", 3.5);
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters (tunable)
    private LoggedNetworkNumber logOuttakeVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0.023549);
    private LoggedNetworkNumber logOuttakeVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.11935);
    private LoggedNetworkNumber logOuttakeVKA = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kA", 0.025517);
    private LoggedNetworkNumber logOuttakeVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", 0.4);
    private LoggedNetworkNumber logOuttakeVKD = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kD", 0.0);
    private LoggedNetworkNumber logOuttakeVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 9450);
    private final int outtakeVSlot = 0;
    private final MotionMagicVelocityVoltage outtakeVRequest;

    // Manual shooter velocity setpoint - tweak this to control the shooter speed
    private LoggedNetworkNumber logTuningShooterVelocity = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity", 40);

    // Flags
    private boolean isZeroingDone = false;
    private boolean hasReachedShooterVelocity = false;

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Shooter", false);

    private double manualShift = 0; // For manually adjusting shooter velocity on the fly, added to getShooterVeloc()

    // Timer
    int slotNum = 0;

    // Data tables (distance -> time, used by the vision subsystem for shot timing)
    InterpolatingDoubleTreeMap shootingDistanceToTime = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ferryingDistanceToTime = new InterpolatingDoubleTreeMap();

    // Cached Values
    private double cachedDistanceToTarget = 0.0;

    // ----------------------------- CONSTRUCTOR ---------------------------- //
    public ShooterSubsystem() {
        configureAngleMotors();
        configureShooterMotors();
        addData();
        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot).withEnableFOC(RobotConstants.ENABLE_FOC);
    }

    public void addData() {
        shootingDistanceToTime.put(5.095, 1.4525);
        shootingDistanceToTime.put(4.101, 1.374);
        shootingDistanceToTime.put(3.465, 1.3325);
        shootingDistanceToTime.put(3.0, 1.235);
        shootingDistanceToTime.put(2.4, 1.1383);
        shootingDistanceToTime.put(1.73, 1.032);
        shootingDistanceToTime.put(1.1, 0.956);

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
        if (!isZeroingDone) {
            currentAngleState = AngleState.ZEROING;
            isZeroingDone = autoZeroing();
        }

        updateReachedShooterVelocityLogic();

        publishLogs();

        applyState();
    }

    public void publishLogs() {
        Logger.recordOutput("Rebuilt/Shooter/Hood/currentState", currentAngleState);
        Logger.recordOutput("Rebuilt/Shooter/Hood/angleSetpointMotorRevs", targetHoodHeight);
        Logger.recordOutput("Rebuilt/Shooter/Hood/current motor revolutions", angleMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Hood/Left Supply Amps", angleMotor.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Shooter/Outtake/currentState", currentShooterState);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/velocitySetpoint", targetShooterVeloc + manualShift);
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Left Supply Amps", shooterLeftMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Left current velocity", shooterLeftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Shooter/Outtake/Manual Adjustment", manualShift);

        if (logDebuggingToggle.get()) { // To get rid of loop overruns
            Logger.recordOutput("Rebuilt/Shooter/Hood/voltage", angleMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Rebuilt/Shooter/Hood/Request Accel", hoodMMRequest.Acceleration);
            Logger.recordOutput("Rebuilt/Shooter/Hood/Request Veloc", hoodMMRequest.Velocity);
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
                angleMotor.setControl(new VoltageOut(-1.0));
                break;

            // Manual control: drive to the tunable hood setpoint (no limelights)
            case AIMING:
            case TUNING:
                angleMotor.setControl(hoodMMRequest.withPosition(getShooterAngle()).withSlot(slotNum));
                break;

            case STORING:
                angleMotor.setControl(hoodMMRequest.withPosition(0).withSlot(slotNum));
                break;

            default:
                break;
        }
    }


    // --------------------------------------- GETTERS --------------------------------------- //

    // Manual shooter speed: pulled straight from the tunable velocity + manual shift
    public double getShooterVeloc() {
        targetShooterVeloc = MathUtil.clamp(logTuningShooterVelocity.get(), 0, 100);
        return targetShooterVeloc + manualShift;
    }

    // Manual hood angle: pulled straight from the tunable target (in motor revs)
    public double getShooterAngle() {
        targetHoodHeight = MathUtil.clamp(logHoodMotorRevTarget.get(), hoodAngleMin, hoodAngleMax);
        return targetHoodHeight;
    }

    public double getShootingDistanceToTime(double distance) {
        return shootingDistanceToTime.get(distance);
    }

    public double getFerryingDistanceToTime(double distance) {
        return ferryingDistanceToTime.get(distance);
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

    public boolean isShooterReady() {
        if(!hasReachedShooterVelocity) {
            return (isAngleWithinTolerance() && isVelocityWithinTolerance());
        }
        return (isAngleWithinTolerance() && isVelocityWithinTolerance(20));
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
        if(wantedAngleState == AngleState.ZEROING) {
            isZeroingDone = false;
        }
        currentShooterState = wantedShooterState;
        currentAngleState = wantedAngleState;
    }

    public Command setStateCommand(ShooterState wantedShooterState, AngleState wantedAngleState) {
        return new InstantCommand(() -> setState(wantedShooterState, wantedAngleState));
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
        boolean updateOuttakeVals = (shooterMotorsConfig.Slot0.kS != logOuttakeVKS.get())
                    || (shooterMotorsConfig.Slot0.kV != logOuttakeVKV.get())
                    || (shooterMotorsConfig.Slot0.kA != logOuttakeVKA.get())
                    || (shooterMotorsConfig.Slot0.kP != logOuttakeVKP.get())
                    || (shooterMotorsConfig.Slot0.kD != logOuttakeVKD.get())
                    || (shooterMotorsConfig.MotionMagic.MotionMagicAcceleration != logOuttakeVAccel.get());

        if (updateOuttakeVals) {
            shooterMotorsConfig.Slot0.kS = logOuttakeVKS.get();
            shooterMotorsConfig.Slot0.kV = logOuttakeVKV.get();
            shooterMotorsConfig.Slot0.kA = logOuttakeVKA.get();
            shooterMotorsConfig.Slot0.kP = logOuttakeVKP.get();
            shooterMotorsConfig.Slot0.kD = logOuttakeVKD.get();

            shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = logOuttakeVAccel.get();

            configureShooterMotors();
        }


        boolean updateAngleVals = (angleMotorConfig.Slot0.kS != logHoodMMKS.get())
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

            hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        }
    }

    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
            instance.initizalize();
        }
        return instance;
    }

    public void initizalize() {
        coordSub = CoordinationSubsystem.getInstance();
    }

    public ArrayList<TalonFX> getMotors() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(angleMotor);
        arrList.add(shooterLeftMotor);
        arrList.add(shooterRightMotor);
        return arrList;
    }

}
