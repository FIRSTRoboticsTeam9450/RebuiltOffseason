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
    // Zeroing, Aiming, Deadzone, Storing, Tuning
    public enum AngleState {
    }

    // Shooting, not shooting
    public enum ShooterState {
    }

    // Other subsystems
    
    private double hoodAngleMax = 14.3;
    private double hoodAngleMin = 0; //old: 3.5
    private double hoodFerryMotorRev = 14.15;
    private double degreesPerRev = HoodConstants.degreesPerMotorRev;
    private double degreesToZero = 8;
    private double distanceAwayToUseLowDegree = 1.2;
    private double distanceAwayToUseHighDegree = 1.78;

    // Motors
    // 2 Shooter motors, 1 Hood angle motor
    
    // Configurators

    // Hood Motion Magic parameters
    // DynamicMotionMagic - Allows FOC
    // Veloc, Accel, S, V, A, P, I, D, G
    private DynamicMotionMagicVoltage hoodMMRequest;

    // Shooter Outtake Velocity Motion Magic Parameters
    // Velocity Voltage S,V,A,P,D, Accel

    private final MotionMagicVelocityVoltage outtakeVRequest;

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
        hoodMMRequest = new DynamicMotionMagicVoltage(0, hoodMMVelocity, hoodMMAcceleration).withEnableFOC(RobotConstants.ENABLE_FOC).withSlot(slotNum);
        outtakeVRequest = new MotionMagicVelocityVoltage(0).withSlot(outtakeVSlot).withEnableFOC(RobotConstants.ENABLE_FOC);

    }

    // Data for interpolated data tree for shooter wheel velocity based on distance away from target
    public void addData() {

    }

    public void configureAngleMotors() {
        // Set the neutral mode
        // Set the way the motor spins for pos (find it through phoenix tunner!)
        // Set current limits
        // Set motion magic configs
        // Software limits (hood angle max!)
        // Give it to the config
        // If you need help refer to your intake subsystem for syntax!
    }

   private void configureShooterMotors() {
        // Current limits 80 for stator current limit, 70 for supply current limit
        // Configure both motors!
        // Make one follow the other (just like in the intake!)
    }

    // Zero the hood motor!
    public boolean autoZeroing() {
        
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
        
        // Apply state
    }

    public void publishLogs() {
        
        if (logDebuggingToggle.get()) { // To get rid of loop overruns
        
        }
    }

    public void applyState() {
        // When shooting, have it's velocity come from the interpolated data tree depending on the distance it's from the target
        
        // Angle states
        // Have the aiming state just be manual control because we dont have limelights
    }


    // --------------------------------------- GETTERS --------------------------------------- //

    // Switch hood revs to degrees
    public double getHoodAngleDegrees() {
        return 0;
    }

    // Switch to revs
    public double getAsRevs(double degrees) {
        return 0;
    }

    // Checks
    // Is velocity in tolerance?
    // Is angle in tolerance?
    // Is the overall shooter ready?
    
    // -------------------------------------- SETTERS -------------------------------------- //
    // Set states
    // -------------------------------------------- UPDATE BASED ON LOGS -------------------------------------------- //
    public void updateShooterConstants() {
        // Check if you need to update hood motor motion magic values, then update it
        // Check if you need to update shooter motors, then update them
        
    }

    // Write get instance


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
