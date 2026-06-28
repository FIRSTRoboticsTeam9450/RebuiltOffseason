package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.TrenchManeuvering;
import frc.robot.Constants.FieldConstants.TrenchManeuvering.TrenchLocationWRTBlueOrigin;
import frc.robot.commands.GoToPosCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.TransferState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.subsystems.VisionSubsystem.ZoneState;
import frc.robot.util.HubTracker;
import frc.robot.util.HubTracker.Shift;

public class CoordinationSubsystem extends SubsystemBase {

    private static volatile CoordinationSubsystem INSTANCE;

    private CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

    /* What do we want Coordsub to do?
     * Have a trench mode where the hood goes down and it stops shooting
     * Have a mode for when the robot isn't ready to shoot yet (systems are still firing up to speed)
     * Have a mode for active shooting
     * Have an initialization mode for when systems are zeroing
     */
    public enum AbsoluteStates {
        /* Make the states! */
    }

    // Current state

    // FLAGS

    private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Coord", false);

    public CoordinationSubsystem() {
        
    }


    @Override
    public void periodic() {
        // What do you call in every subsystem!

    }

    private void applyState() {
        
        // Account for each state, think about what you want the robot to do in each state?
        // Intake has a lot of specialized states as it could be either intaking or not at any point
        // It's better to handle the intake with flags rather than individualized states for mode
        /* Your code here */

        // Set the states for each subsytem
    }

    private void publishLogs() {
        // Add any logs you need here!

        Logger.recordOutput("Rebuilt/Match Data/Our Hub Active?", HubTracker.isActive());
        Logger.recordOutput("Rebuilt/Match Data/We Win Auto?", DriverStation.getAlliance().isPresent() && HubTracker.getAutoWinner().isPresent() ? DriverStation.getAlliance().get() == HubTracker.getAutoWinner().get() : false);
        Logger.recordOutput("Rebuilt/Match Data/Time Left Until Shift Change", HubTracker.timeRemainingInCurrentShift().isPresent() ? HubTracker.timeRemainingInCurrentShift().get() : -1);
        Logger.recordOutput("Rebuilt/Match Data/Shift", HubTracker.getCurrentShift().isPresent() ? HubTracker.getCurrentShift().get() : Shift.AUTO);


        if(logDebuggingToggle.get()) {
        }
    }

    // ======================== Conditions =============================
    public boolean isZeroingDone() {
        // Check if everything is done zeroing
        // Don't forget to call it! (hint: how often do you think you would want to check this?)
        /* Your code here */
        return false;
    }

    // Check if the shooter is ready, if the turret is ready and if the driver wants the robot to fire
    // Write methods to check!
    /* Your code here */

    // 
    public boolean goodToFire() {
        /* Your code here */
        return false;
    }


    // ============================ Getters & Setters ============================

    // Allow the driver to set fire, set the intake, rejecting, override trench
    // Set state

    // ======================= INIT ===================

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
            INSTANCE.initizalize();
        }
        return INSTANCE;
    }

    public void initizalize() {
    }
}
