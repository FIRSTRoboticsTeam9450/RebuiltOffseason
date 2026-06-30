package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TransferConstants;

/**
 * Subsystem which controls the handoff from Intake to Shooter.
 */
public class TransferSubsystem extends SubsystemBase {
    /** Transfer subsystem instance */
    private static volatile TransferSubsystem INSTANCE;

    /**
     * State machine for transfer operation modes
     */
    // States include: not running, running out, running in, fast run in, holding
    public enum TransferState {
    }
    
    // Record you current state

    // One motor! ID: 25, on the main canbus
    

    /** Voltage request */
    private VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private Timer stateTimer = new Timer();

    private double jammedCount = 0;

    /**
     * Constructor - initializes the transfer subsystem and configures the motor
     */
    private TransferSubsystem() {
        configureMotor();
    }

    /**
     * Returns the single instance of the TransferSubsystem
     * @return The TurretSubsystem instance
     */
    public static synchronized TransferSubsystem getInstance() {
        // Check if there's already an instance if so, return that instance
        // If there isn't, create a new instance
        return null; // <- Replace this!
    }

    private void initialize() {
        //Logger.recordOutput("Rebuilt/Transfer/Subsystem Initizalized", true);
    }

    /**
     * Configures the TalonFX motor
     */
    private void configureMotor() {
        // Coast
        // Clockwise Positive
        // Stator current limit = 80, Supply currrent limit = 50

    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        applyState();
        publishLogs();
    }

    /**
     * State machine implementation - executes logic for current state
     */
    private void applyState() {
        // Set request voltage
        // Depending on the state, you will want different voltages
        // For example, HOLD just wants to hold the fuel in so a very small voltage would work
        // To set the output of the voltagerequest: .output = 
        // Set the control to be the voltage request
    }



    // Set state methods:

    /**
     * Publishes telemetry data to AdvantageKit Logger
     */
    private void publishLogs() {
    }

    public TalonFX getMotor() {
        return null;
    }
}
