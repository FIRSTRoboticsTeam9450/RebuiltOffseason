package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignPos;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The goal of this command is to use our auto align system without the need of seeing an april tag
 */
public class PositionAlignCommand extends Command {


    /*   ─────────────────────────── 
     *  |          Fields           |
     *   ─────────────────────────── 
     */ 

     
    /* --------------- Subsystem Instances --------------- */
    CommandSwerveDrivetrain drivetrain;

    /* --------------- PIDs --------------- */
    private PIDController pidX = new PIDController(2.5, 0, 0.0);
    private PIDController pidY = new PIDController(2.5, 0, 0.0);
    private PIDController pidR = new PIDController(2.5, 0, 0.0);

    /* --------------- Timer --------------- */
    private Timer timer = new Timer();

    /* --------------- Variables --------------- */
    private AlignPos alignSide;
    private Pose2d targetPose;
    private Pose2d currentFieldPose;
    private boolean redAlliance;
    private double[] offsetArr = new double[3];
    private double tagLeftOffset = 0.0;

    /* --------------- Debugging Variables --------------- */
    private double debuggingXPower;
    private double debuggingYPower;
    private double debuggingRPower;

    /* --------------- Drive Request --------------- */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                                                                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                                                             .withRotationalDeadband(0.05)
                                                                             .withDeadband(0.05);


    /*   ─────────────────────────── 
     *  |       Initialization      |
     *   ─────────────────────────── 
     */ 


    /**
     * Configures instances of drive subsystem, which pole we want to go to, and april tag locations
     * @param drivetrain
     * @param alignPos
     */
    public PositionAlignCommand(CommandSwerveDrivetrain drivetrain, AlignPos alignSide) {
        this.drivetrain = drivetrain;
        this.alignSide = alignSide;

        pidR.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        redAlliance = !FieldConstants.isBlueAlliance;

        debuggingXPower = 0;
        debuggingYPower = 0;
        debuggingRPower = 0;

        timer.restart();

        if (!redAlliance) {
            targetPose = new Pose2d(FieldConstants.ImportantFieldPoses.BLUE_TOWER, new Rotation2d(0));
        } else {
            targetPose = new Pose2d(FieldConstants.ImportantFieldPoses.RED_TOWER, new Rotation2d(Math.PI));
        }


        offsetArr = calculateAlignPosition(targetPose, 0.75);
        pidX.setSetpoint(offsetArr[0]);
        pidY.setSetpoint(offsetArr[1]);
        pidR.setSetpoint(offsetArr[2]);
    }


    /*   ─────────────────────────── 
     *  |      Periodical loop      |
     *   ─────────────────────────── 
     */ 


    @Override
    public void execute() {

        currentFieldPose = drivetrain.getState().Pose;

        /* ---------- Scoot ---------- */
        if (atSetpoint(0.075, 0.25)) {
            offsetArr = calculateAlignPosition(targetPose, 0.35);
            pidX.setSetpoint(offsetArr[0]);
            pidY.setSetpoint(offsetArr[1]);
            pidR.setSetpoint(offsetArr[2]);
        }


         /* --------------- Scoring --------------- */

        setSwerveRequest(
            calculateDrivePower());

            publishLogs();
    }


    /*   ─────────────────────────── 
     *  |        Calculations       |
     *   ─────────────────────────── 
     */ 


    /**
     * Calculate the relative Distance and rotational errors from robot position to alliance tower
     * @param targetPose 2D pose of alliance tower
     * @param tagForwardOffset Forward/Backward Distance away from tower pole we wish to be at (Meters)
     * @return Double array which holds the distance from robot to tower pole in (X, Y, Rot)
     */
    private double[] calculateAlignPosition(Pose2d targetPose, double tagForwardOffset) {

        /* --------------- Offset Calculations --------------- */
        //Calculate how far in and to the side we wish to be relative to the Tower
        switch (alignSide) {
            case LEFT:
                tagLeftOffset = -0.425; //-0.5
                break;

            case RIGHT:
                tagLeftOffset = 0.425; //0.5
                break;

            default:
                tagLeftOffset = 0;
                break;
        }


        /* --------------- Setpoint Calculations --------------- */

        //Calculate Rotational Error from robot to reef side
        double rotationalSetpoint = targetPose.getRotation().getRadians();
        rotationalSetpoint -= Math.PI;

        //Normalize Error to ensure it wraps for shortest distance
        if (rotationalSetpoint < -Math.PI) {
            rotationalSetpoint += 2 * Math.PI;
        } else if (rotationalSetpoint > Math.PI) {
            rotationalSetpoint -= 2 * Math.PI;
        }

        //Calculate X & Y Error from robot to April Tag of Reef Side
        double xSetpoint = targetPose.getX() - tagForwardOffset * Math.cos(rotationalSetpoint) - tagLeftOffset * Math.sin(rotationalSetpoint);
        double ySetpoint = targetPose.getY() - tagForwardOffset * Math.sin(rotationalSetpoint) + tagLeftOffset * Math.cos(rotationalSetpoint);

        rotationalSetpoint = !redAlliance ? 1.5 * Math.PI : 0.5 * Math.PI;

        double[] out = {xSetpoint, ySetpoint, rotationalSetpoint};

        Pose2d offsetAugmentedPose = new Pose2d(new Translation2d(xSetpoint, ySetpoint), new Rotation2d(rotationalSetpoint));

        // Logger.recordOutput("Rebuilt/Pose-Align/Robot go to Spot", offsetAugmentedPose);

        return out;
    }

    /**
     * Used to keep main methods like initialize and execute clean and organized.
     * <p>
     * Calculates the power in which the drive request should use
     * </p>
     * @param pidSetpoints Offsets that the CalculateAlignPositions method should have given
     * @return Double array of which the drive request should use (xVeloc, yVeloc, rotVeloc)
     */
    private double[] calculateDrivePower() {

        //used for acceleration
        double time = timer.get();

        //Powers in which the drive request will use
        double xPower = pidX.calculate(currentFieldPose.getX()) * (MathUtil.clamp(time, 1, 0));
        double yPower = pidY.calculate(currentFieldPose.getY()) * (MathUtil.clamp(time, 1, 0));

        //Adjustments of the power depending on certain conditions
        xPower = MathUtil.clamp(xPower, -3.5, 3.5);
        yPower = MathUtil.clamp(yPower, -3.5, 3.5);

        xPower += .05*Math.signum(xPower);
        yPower += .05*Math.signum(yPower);

        double rPower = pidR.calculate(currentFieldPose.getRotation().getRadians());
        rPower = MathUtil.clamp(rPower, -5, 5);

        //Flips power depending on alliance due to IMU being different
        if (redAlliance) {
            xPower *= -1;
            yPower *= -1;
        }

        double[] out = {xPower, yPower, rPower};

        debuggingXPower = out[0];
        debuggingYPower = out[1];
        debuggingRPower = out[2];
        
        return out;
    }


    /*   ─────────────────────────── 
     *  |       Swerve Request      |
     *   ─────────────────────────── 
     */


    /**
     * Used for having a clean way to set Swerve with just needing to provide a previously calculated array
     * @param powerArr array which contains calculated power values
     */
    private void setSwerveRequest(double[] powerArr) {

        String debuggingString = "";

        for (double elem : powerArr) {
            debuggingString += elem + ", ";
        }

        //Logger.recordOutput("Rebuilt/Pose-Align/Swerve Request Debug", debuggingString);

        SwerveRequest swerveRequest = driveRequest
                                            .withVelocityX(powerArr[0])
                                            .withVelocityY(powerArr[1])
                                            .withRotationalRate(powerArr[2]);
        
        drivetrain.setControl(swerveRequest);
    }


    /*   ─────────────────────────── 
     *  |    Setpoint Tolerences    |
     *   ─────────────────────────── 
     */ 


    public boolean atSetpoint() {
        return atSetpoint(0.03, 0.2);
    }

    public boolean atSetpoint(double translationTolerance, double rotationTolerance) {
        return Math.abs(pidX.getSetpoint() - currentFieldPose.getX()) < translationTolerance && Math.abs(pidY.getSetpoint() - currentFieldPose.getY()) < translationTolerance && pidR.getError() < rotationTolerance;
    }


    /*   ─────────────────────────── 
     *  |         Debugging         |
     *   ─────────────────────────── 
     */ 

    private void publishLogs() {

        //Field Pose we use for calculations
        //Logger.recordOutput("Rebuilt/Pose-Align/Assumed Field Pose", currentFieldPose);

        //Logger.recordOutput("Rebuilt/Pose-Align/Align Side", alignSide);
        //Logger.recordOutput("Rebuilt/Pose-Align/Align Offset", tagLeftOffset);

        //Different powers calculated off of offsets and other conditions
        //Logger.recordOutput("Rebuilt/Pose-Align/Drive Power/X Power", debuggingXPower);
        //Logger.recordOutput("Rebuilt/Pose-Align/Drive Power/Y Power", debuggingYPower);
        //Logger.recordOutput("Rebuilt/Pose-Align/Drive Power/Rotation Power", debuggingRPower);

        //Logger.recordOutput("Rebuilt/Pose-Align/Current Drive Command", drivetrain.getCurrentCommand().getName());

        //Distance to targeted reef side
        // //Logger.recordOutput("Rebuilt/Pose-Align/Distance to Target", Math.sqrt(Math.pow((debuggingTargetPose.getX() - currentFieldPose.getX()), 2) + Math.pow((debuggingTargetPose.getY() - currentFieldPose.getY()), 2)));

    }


    /*   ───────────────────────────
     *  |            End            |
     *   ───────────────────────────
     */ 


     /**
     * Ends the current drive control, either when the command is finished or interrupted.
     *
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

        // Set the drive control with the stop request to halt all movement
        drivetrain.setControl(stop);
    }

}