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
    public enum TransferState {
        HOLD,
        RUN_IN,
        SLOW_RUN_IN,
        FAST_RUN_IN,
        HOPPER_RUN_OUT,
        RUN_OUT,
        NO_RUN
    }
    private TransferState currentState = TransferState.NO_RUN;
    private TransferState previousState = currentState;
    // Horizontal Roller one or 2
    // 1 vertical roller
    // Side sweeper - top and bottom motors
    // Tower = other motor
    private final TalonFX motorTower = new TalonFX(TransferConstants.DeviceIDs.kTowerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private final TalonFX motorHorizontalRoller = new TalonFX(TransferConstants.DeviceIDs.kHorizontalRollerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private final TalonFX motorVerticalRoller = new TalonFX(TransferConstants.DeviceIDs.kVerticalRollerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private final TalonFX motorTopSideSweeper = new TalonFX(TransferConstants.DeviceIDs.kVerticalRollerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    private final TalonFX motorBottomSideSweeper = new TalonFX(TransferConstants.DeviceIDs.kVerticalRollerMotorID, RobotConstants.MAIN_SYSTEMS_CANBUS);
    
    /** Voltage request */
    private VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(RobotConstants.ENABLE_FOC);

    private Timer stateTimer = new Timer();

    private double jammedCount = 0;
    private double towerVoltSetpoint = 0;
    private double horizontalRollVoltSetpoint = 0;
    private double verticalRollVoltSetpoint = 0;
    private double topSweepVoltSetpoint = 0;
    private double bottomSweepVoltSetpoint = 0;

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
        if (INSTANCE == null) {
            INSTANCE = new TransferSubsystem();
            INSTANCE.initialize();
        }
        return INSTANCE;
    }

    private void initialize() {
        //Logger.recordOutput("Rebuilt/Transfer/Subsystem Initizalized", true);
    }

    /**
     * Configures the TalonFX motor
     */
    private void configureMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        
        motorTower.getConfigurator().apply(motorConfig);
    }

    /**
     * Periodic update function called every 20ms by the command scheduler
     */
    @Override
    public void periodic() {
        checkStall();
        applyState();
        publishLogs();
    }

    /**
     * State machine implementation - executes logic for current state
     */
    private void applyState() {
        towerVoltSetpoint = 0;
        horizontalRollVoltSetpoint = 0;
        verticalRollVoltSetpoint = 0;
        topSweepVoltSetpoint = 0;
        bottomSweepVoltSetpoint = 0;

        // Set request voltage
        switch (currentState) {
            case HOLD: // Very slowly move fuel in
                horizontalRollVoltSetpoint = 0.55;
                verticalRollVoltSetpoint = 0.55;
                break;

            case RUN_IN: // Move fuel in
                towerVoltSetpoint = 8;
                horizontalRollVoltSetpoint = 8;
                verticalRollVoltSetpoint = 8;
                topSweepVoltSetpoint = 8;
                bottomSweepVoltSetpoint = 8;
                break;
            
            case SLOW_RUN_IN:
                voltageRequest.Output = 4.5;
                break;

            case FAST_RUN_IN:
                towerVoltSetpoint = 12;
                horizontalRollVoltSetpoint = 12;
                verticalRollVoltSetpoint = 12;
                topSweepVoltSetpoint = 12;
                bottomSweepVoltSetpoint = 12;
                break;
            case RUN_OUT: // Move fuel out
                double voltage = -12 / (stateTimer.get() * 1.23);
                voltage = MathUtil.clamp(voltage, -12, -4);
                towerVoltSetpoint = voltage;
                horizontalRollVoltSetpoint = voltage;
                verticalRollVoltSetpoint = voltage;
                topSweepVoltSetpoint = voltage;
                bottomSweepVoltSetpoint = voltage;
                break;
            case HOPPER_RUN_OUT:
                double voltageHopper = -12 / (stateTimer.get() * 1.23);
                voltageHopper = MathUtil.clamp(voltageHopper, -12, -4);
                horizontalRollVoltSetpoint = voltageHopper;
                verticalRollVoltSetpoint = voltageHopper;
                break;

            case NO_RUN: // Don't move
                voltageRequest.Output = 0;
                break;
        }
        motorTower.setControl(voltageRequest.withOutput(towerVoltSetpoint));
        motorHorizontalRoller.setControl(voltageRequest.withOutput(horizontalRollVoltSetpoint));
        motorVerticalRoller.setControl(voltageRequest.withOutput(verticalRollVoltSetpoint));
        motorBottomSideSweeper.setControl(voltageRequest.withOutput(bottomSweepVoltSetpoint));
        motorTopSideSweeper.setControl(voltageRequest.withOutput(topSweepVoltSetpoint));
    }

    public void checkStall() {
        if(currentState == TransferState.SLOW_RUN_IN || currentState == TransferState.RUN_IN ) {
            if(Math.abs(voltageRequest.Output) > 4 && Math.abs(motorTower.getVelocity().getValueAsDouble()) < 1) {
                jammedCount++;
            }
            else {
                jammedCount = 0;
            }
        }
        else {
            jammedCount = 0;
        }
    }

    public boolean jammed() {
        if(jammedCount > 50) {
            return true;
        }
        else {
            return false;
        }
    }


    /**
     * Changes the current state of the transfer
     * @param wantedState The desired state to transition to
     */
    public void setState(TransferState wantedState) {
        stateTimer.restart();
        previousState = currentState;
        currentState = wantedState;
    }

    /**
     * Get a command that sets the current state of the transfer
     * @param wantedState The desired state to transition to
     */
    public Command setStateCommand(TransferState wantedState) {
        return new InstantCommand(() -> setState(wantedState));
    }

    /**
     * Publishes telemetry data to AdvantageKit Logger
     */
    private void publishLogs() {
        Logger.recordOutput("Rebuilt/Transfer/Current State", currentState);
        // Logger.recordOutput("Rebuilt/Transfer/Previous State", previousState);

        // Logger.recordOutput("Rebuilt/Transfer/Target Speed", voltageRequest.Output);

        // Logger.recordOutput("Rebuilt/Transfer/Stalling", jammed());

        Logger.recordOutput("Rebuilt/Transfer/Tower Motor Voltage", motorTower.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Transfer/Horizontal Motor Voltage", motorHorizontalRoller.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Transfer/Vertical Motor Voltage", motorVerticalRoller.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Transfer/Top Side Motor Voltage", motorTopSideSweeper.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Rebuilt/Transfer/Bottom Side Motor Voltage", motorBottomSideSweeper.getMotorVoltage().getValueAsDouble());

        // Logger.recordOutput("Rebuilt/Transfer/Motor Veloc", motor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Transfer/Motor Supply Amps", motor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Rebuilt/Transfer/Motor Stator Amps", motor.getTorqueCurrent().getValueAsDouble());

        Logger.recordOutput("Rebuilt/Transfer/Transfer Jammed", jammed());
    }

    public ArrayList<TalonFX> getMotor() {
        ArrayList<TalonFX> arrList = new ArrayList<>();
        arrList.add(motorBottomSideSweeper);
        arrList.add(motorHorizontalRoller);
        arrList.add(motorTopSideSweeper);
        arrList.add(motorTower);
        arrList.add(motorVerticalRoller);
        return arrList;
    }
}
