package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants.TurretConstants;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.BackRight;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimelightConstants.FrontRight;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ImportantFieldPoses;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneConstantsWelded;
import frc.robot.Constants.FieldConstants.ZoneConstants.ZoneTargets;
import frc.robot.util.BezierCurve;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightHelpers.IMUData;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.LimelightObject.Location;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * VisionSubsystem using Limelights with:
 *  - Fusion of multiple cameras
 *  - Singleton pattern
 *  - Integration with drivetrain for pose estimation
 *
 * This subsystem reads Limelight data, fuses multiple camera poses,
 * and provides utility functions like distance to target and shooter recommendations.
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------
    // Singleton instance
    // -------------------------
    private static volatile VisionSubsystem instance;

    /**
     * Singleton accessor method.
     * Ensures only one instance exists and is globally accessible.
     *
     * @param drivetrain Drivetrain instance for updating pose with vision
     * @return VisionSubsystem instance
     */
    public static synchronized VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
            instance.initizalize();
        }
        return instance;
    }

    public void initizalize() {
        
    }

    // -------------------------
    // Core subsystem fields
    // -------------------------
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain; // Reference to drivetrain for pose updates

    private volatile Pose2d lastVisionPose = drivetrain.getState().Pose;  // Last pose estimated from fused vision
    private volatile double lastVisionTimestamp = 0.0; // Timestamp of last vision update

    // Standard deviations for vision measurements (for filtering/fusion)
    public Vector<N3> disabledStdDev = VecBuilder.fill(3, 3, 999999);
    public Vector<N3> enabledMT2StdDev = VecBuilder.fill(0.3, 0.3, 999999);
    public Vector<N3> enabledMT1StdDev = VecBuilder.fill(999999, 999999, 1);
    private Vector<N3> badOdomStdDev = VecBuilder.fill(0, 0, 999999);

    public double defaultVisionStdDevX = enabledMT2StdDev.get(0);
    public double defaultVisionStdDevY = enabledMT2StdDev.get(1);
    public double defaultVisionStdDevTheta = enabledMT2StdDev.get(2);
    public double currentVisionStdDevX = 0;
    public double currentVisionStdDevY = 0;
    private double currentVisionStdDevTheta = 0.0;
    private double updateVisionOffset = 0;
    private double updateTargetAndZoneOffset = 0;

    private boolean bumpDetected = false;
    private double bumpMeasurmentCount = 5;
    private boolean odomPoseOutsideField = false;
    private boolean operatorForcedVisionTrust = false;

    // Maximum allowed differences for fusion; if exceeded, fallback to higher confidence camera
    private double maxTranslationDiff = 2.5; // meters
    private double maxRotationDiff = Math.toRadians(45); // radians

    // Validation thresholds for pose filtering
    private static final double MAX_Z_THRESHOLD = 0.25; // Robot must be on ground
    private static final double MAX_PITCH_THRESHOLD = 1000; // Robot must not be tilted forward/back
    private static final double MAX_ROLL_THRESHOLD = 1000; // Robot must not be tilted side-to-side
    private static final double MAX_MT2_YAW_DIFF_DEGREES = 0.5; // Vision yaw must match Pigeon within MAX_YAW_DIFF degrees
    private static final double MAX_MT1_YAW_DIFF_DEGREES = 2.25;

    // Limelight camera objects
    private final List<LimelightObject> LIMELIGHT_LIST = new ArrayList<>();

    // Logging
    private volatile Pose2d lastFusedPose = null;   // Last fused pose from vision

    private ChassisSpeeds prevRobotVelocity = new ChassisSpeeds(0, 0, 0);
    private double prevTimeStamp = 0;


    // DEBUGGING
    private LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("/Tuning/Debugging Toggles/Vision", false);
    double loopOffset = 0;

    private final CommandXboxController DRIVER = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
    // -------------------------
    // Constructor (private for singleton)
    // -------------------------

    private VisionSubsystem() {
        drivetrain = RobotContainer.drivetrain;

        // Configure camera offsets and add to internal list
        configureLimelights();

        // Initialize drivetrain vision standard deviations
        setVisionMeasurementStdDevs(VecBuilder.fill(defaultVisionStdDevX, defaultVisionStdDevY, defaultVisionStdDevTheta));
    }

    /**
     * Initializes LimelightObject instances with physical offsets and names.
     * Ensures that all camera poses are correctly transformed to robot frame.
     */
    private void configureLimelights() {
        // Front Left Limelight
        LimelightObject limelightFL = new LimelightObject(FrontLeft.NAME)
                                        .withLocation(Location.FRONT_LEFT)
                                        .withPose3d(FrontLeft.POSE);
        LIMELIGHT_LIST.add(limelightFL);

        // Front Right Limelight
        LimelightObject limelightFR = new LimelightObject(FrontRight.NAME)
                                        .withLocation(Location.FRONT_RIGHT)
                                        .withPose3d(FrontRight.POSE);
        LIMELIGHT_LIST.add(limelightFR);

        // Back Left Limelight
        LimelightObject limelightBL = new LimelightObject(BackLeft.NAME)
                                        .withLocation(Location.BACK_LEFT)
                                        .withPose3d(BackLeft.POSE);
        LIMELIGHT_LIST.add(limelightBL);

        // Back Right Limelight
        LimelightObject limelightBR = new LimelightObject(BackRight.NAME)
                                        .withLocation(Location.BACK_RIGHT)
                                        .withPose3d(BackRight.POSE);
        LIMELIGHT_LIST.add(limelightBR);
    }

    /**
     * Periodic method called every ~20ms by the scheduler.
     * Updates vision data and logs key metrics.
     */
    @Override
    public void periodic() {

        if (updateVisionOffset > 1) {
            updateVisionOffset = 0;
        } else {
            updateVisionOffset++;
        }

        updateVision();  // Compute fused vision pose
        publishLogs();   // Print/log fusion confidence and pose
    }

    // -------------------------
    // Vision update + fusion
    // -------------------------

    /**
     * Updates pose estimates from all Limelight cameras and fuses them.
     * Sends fused pose to drivetrain for odometry correction.
     */
    private void updateVision() {

        Pose2d robotPoseData = drivetrain.getState().Pose;
        Pigeon2 pigeonData = drivetrain.getPigeon2();

        double robotTranslationX = robotPoseData.getX();
        double robotTranslationY = robotPoseData.getY();

        double robotYaw = robotPoseData.getRotation().getDegrees();
        double robotYawRate = pigeonData.getAngularVelocityZDevice().getValueAsDouble();
        double robotRoll = Math.abs(pigeonData.getRoll().getValueAsDouble());
        double robotPitch = Math.abs(pigeonData.getPitch().getValueAsDouble());

        if (robotTranslationX < 0 || robotTranslationX > 16.575 || robotTranslationY < 0 || robotTranslationY > 8.1) {
            odomPoseOutsideField = true;
        }

        if (robotRoll > 6 || robotPitch > 6) {
            bumpDetected = true;
            bumpMeasurmentCount = 0;
        }

        // Logger.recordOutput("Rebuilt/Vision/Odom Pose outside Field", odomPoseOutsideField);
        // Logger.recordOutput("Rebuilt/Vision/Bump Detected", bumpDetected);

        List<limelightNetworkTableData> llDataList = new ArrayList<>(LIMELIGHT_LIST.size());
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            String name = limelight.getName();
            LimelightHelpers.SetRobotOrientation_NoFlush(name, robotYaw, robotYawRate, 0, 0, 0, 0);
            boolean hasTarget = LimelightHelpers.getTV(name);

            if (hasTarget) {
                llDataList.add(new limelightNetworkTableData(
                    name,
                    true,
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name),
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(name),
                    LimelightHelpers.getBotPose_wpiBlue(name),
                    LimelightHelpers.getRawFiducials(name),
                    LimelightHelpers.getIMUData(name)
                ));
            } else {
                llDataList.add(new limelightNetworkTableData(name, false, null, null, null, null, null));
            }
        }

        for (limelightNetworkTableData llData : llDataList) {
            if (llData.megaTag1Estimate != null && llData.megaTag1Estimate.tagCount >= 1) {
                double sampleRobotYaw = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(llData.megaTag1Estimate.timestampSeconds)).isPresent() ? drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(llData.megaTag1Estimate.timestampSeconds)).get().getRotation().getDegrees() : 100000;
                double limelightYaw = llData.megaTag1Estimate.pose.getRotation().getDegrees();
                if (Math.abs(sampleRobotYaw - limelightYaw) < MAX_MT1_YAW_DIFF_DEGREES) {
                    setVisionMeasurementStdDevs(enabledMT1StdDev);
                    drivetrain.addVisionMeasurement(llData.megaTag1Estimate.pose, llData.megaTag1Estimate.timestampSeconds);
                }
            }
        }

        // === STEP 2: Process snapshots — no more NT reads below this line ===
        List<VisionMeasurement> measurements = new ArrayList<>(llDataList.size());
        for (limelightNetworkTableData snap : llDataList) {
            if (!snap.hasATarget) continue;
            VisionMeasurement vm = createVisionMeasurement(snap);
            if (vm.isValid()) {
                measurements.add(vm);
                // Logger.recordOutput("Rebuilt/Vision/" + snap.name + "/Pose 3D", snap.megaTagEstimate.pose);
            }
        }

        // === STEP 3: Fuse ===
        Optional<Pose2d> fusedPose = fuseVision(measurements);
        fusedPose.ifPresent(pose -> {
            lastVisionPose = pose;
            lastFusedPose = pose;
            lastVisionTimestamp = measurements.stream()
                .mapToDouble(VisionMeasurement::timestamp)
                .max().orElse(0.0);
            if (odomPoseOutsideField || operatorForcedVisionTrust) {
                odomPoseOutsideField = false;
                setVisionMeasurementStdDevs(badOdomStdDev);
            } else if (bumpDetected || bumpMeasurmentCount <= 5) {
                if (bumpDetected) {
                    bumpDetected = false;
                    setVisionMeasurementStdDevs(badOdomStdDev);
                } else {
                    double stdDev = 0.4 * bumpMeasurmentCount;
                    setVisionMeasurementStdDevs(stdDev, stdDev, badOdomStdDev.get(2));
                    bumpMeasurmentCount++;
                }
            } else {
                setVisionMeasurementStdDevs(defaultVisionStdDevX, defaultVisionStdDevY, defaultVisionStdDevTheta);
            }
            drivetrain.addVisionMeasurement(pose, Utils.currentTimeToFPGATime(lastVisionTimestamp));

        });

        // Store snapshots for logging to reuse — no re-reads needed
        // Logger.recordOutput("Rebuilt/Vision/ValidCameraCount", measurements.size());
    }

    /**
     * Converts a Limelight PoseEstimate into a VisionMeasurement.
     * Includes confidence calculation and validity check with pose filtering.
     * 
     * Pose is considered invalid if:
     * - Z position is not zero (robot airborne or on terrain)
     * - Pitch rotation is not zero (robot tilting forward/backward)
     * - Roll rotation is not zero (robot tilting side-to-side)
     * - Yaw differs from Pigeon IMU by more than 0.01 degrees
     */
    private VisionMeasurement createVisionMeasurement(limelightNetworkTableData llData) {
        if (llData == null || llData.megaTag2Estimate == null) return VisionMeasurement.invalid();

        double ambiguity = computeAmbiguity(llData);
        double timestamp = Utils.fpgaToCurrentTime(llData.megaTag2Estimate.timestampSeconds);
        int tagCount = llData.megaTag2Estimate.tagCount;

        if (tagCount == 0 || ambiguity > 0.8) return VisionMeasurement.invalid();
        
        boolean poseValid = true;
        boolean llIMUValid = true;
        
        if (llData.megaTag2Estimate != null) {
            // Extract 3D pose data from botpose array
            // botpose format: [x, y, z, roll, pitch, yaw, ...]
            Pose2d botpose = llData.megaTag2Estimate.pose;
            double[] llPose = {llData.imuData.Roll, llData.imuData.Pitch};
            
            if (botpose != null) {
                double x = botpose.getX();
                double y = botpose.getY();
                double visionYaw = botpose.getRotation().getDegrees();

                double llRoll = llPose[0];
                double llPitch = llPose[1];
                
                // Get current Pigeon yaw
                double pigeonYaw;
                if(!drivetrain.samplePoseAt(timestamp).isPresent()) {
                    pigeonYaw = 100000000;
                }
                else {
                    pigeonYaw = drivetrain.samplePoseAt(timestamp).get().getRotation().getDegrees();

                }
                
                // Validation checks
                boolean xValid = (x > 0) && (x < 16.565);
                boolean yValid = (y > 0) && (y < 8.095);

                boolean llRollValid = Math.abs(llRoll) < 20;
                boolean llPitchValid = Math.abs(-28.1 - llPitch) < 20;
                
                
                // Yaw difference check (handle angle wrapping)
                double yawDiff = Math.abs(visionYaw - pigeonYaw);
                // Normalize to [-180, 180]
                while (yawDiff > 180) yawDiff -= 360;
                while (yawDiff < -180) yawDiff += 360;
                boolean yawValid = true;
                if (DriverStation.isEnabled()) {
                    yawValid = Math.abs(yawDiff) <= MAX_MT2_YAW_DIFF_DEGREES;
                }
                
                // Pose is only valid if all checks pass
                poseValid = xValid && yValid && yawValid;
                // llIMUValid = llRollValid && llPitchValid;
                llIMUValid = true;
                
                // Optional: Log which check failed for debugging
                if (!poseValid) {
                    if (!yawValid) Logger.recordOutput("Rebuilt/Vision/" + llData.name + "/InvalidYawDiff", yawDiff);
                }
            }
        }

        // Consider measurement valid if at least 1 tag, low ambiguity, AND pose validation passes
        boolean valid = tagCount > 0 && poseValid && llIMUValid && ambiguity < 0.5;

        // Confidence weighted: 70% ambiguity, 30% tag count
        double tagFactor = Math.min(tagCount / 2.0, 1.0);
        double confidence = (1.0 - ambiguity) * 0.7 + tagFactor * 0.3;

        return new VisionMeasurement(llData.megaTag2Estimate.pose, confidence, timestamp, valid);
    }

    /**
     * Computes the best (lowest) ambiguity among all fiducials seen by a Limelight.
     * @Returns 1.0 if no fiducials detected (completely ambiguous).
     */
    private double computeAmbiguity(limelightNetworkTableData llData) {
        LimelightHelpers.RawFiducial[] fiducials = llData.fiducials;
        if (fiducials == null || fiducials.length == 0) return 1.0;

        double best = 1.0;
        for (LimelightHelpers.RawFiducial f : fiducials) {
            if (f == null) continue;
            best = Math.min(best, f.ambiguity);
        }
        return best;
    }

    /**
     * Fuses multiple VisionMeasurements into a single Pose2d.
     * 
     * Algorithm:
     * 1. If no valid measurements, return empty
     * 2. If only one valid measurement, return it
     * 3. If multiple measurements:
     *    a. Compute mean pose
     *    b. Reject outliers that differ too much from mean
     *    c. Fuse remaining measurements using confidence weighting
     * 
     * @param measurements List of vision measurements to fuse
     * @return Optional containing fused pose, or empty if no valid measurements
     */
    private Optional<Pose2d> fuseVision(List<VisionMeasurement> measurements) {
        // Filter valid
        int validCount = 0;
        double sumX = 0, sumY = 0;
        VisionMeasurement best = null;

        for (VisionMeasurement m : measurements) {
            if (!m.isValid()) continue;
            sumX += m.pose().getX();
            sumY += m.pose().getY();
            if (best == null || m.confidence() > best.confidence()) best = m;
            validCount++;
        }

        if (validCount == 0) return Optional.empty();
        if (validCount == 1) return Optional.of(best.pose());

        double meanX = sumX / validCount;
        double meanY = sumY / validCount;

        // Single pass for inlier filtering + weighted fusion
        double totalConf = 0, fusedX = 0, fusedY = 0;
        int inlierCount = 0;
        VisionMeasurement highestConfInlier = null;

        for (VisionMeasurement m : measurements) {
            if (!m.isValid()) continue;
            double dx = m.pose().getX() - meanX;
            double dy = m.pose().getY() - meanY;
            double dist = Math.sqrt(dx*dx + dy*dy);
            double rotDiff = Math.abs(m.pose().getRotation().minus(
                new Rotation2d(Math.atan2(meanY, meanX))).getRadians()); // or use first pose rotation

            if (dist < maxTranslationDiff && rotDiff < maxRotationDiff) {
                fusedX += m.pose().getX() * m.confidence();
                fusedY += m.pose().getY() * m.confidence();
                totalConf += m.confidence();
                inlierCount++;
                if (highestConfInlier == null || m.confidence() > highestConfInlier.confidence())
                    highestConfInlier = m;
            }
        }

        if (inlierCount == 0) return Optional.of(best.pose());
        if (inlierCount == 1) return Optional.of(highestConfInlier.pose());

        double rotation = highestConfInlier.pose().getRotation().getRadians();
        return Optional.of(new Pose2d(fusedX / totalConf, fusedY / totalConf, new Rotation2d(rotation)));
    }

    // -------------------------
    // Logging
    // -------------------------
    private void publishLogs() {
        // Overall vision state
        Logger.recordOutput("Rebuilt/Vision/Total/Last Vision Pose", lastVisionPose);
        // Logger.recordOutput("Rebuilt/Vision/Total/Last Fused Pose", lastFusedPose);
        // Logger.recordOutput("Rebuilt/Vision/Total/Last Target Pose", lastTargetPose);
        // Logger.recordOutput("Rebuilt/Vision/Total/Last Vision Timestamp", lastVisionTimestamp);
        
        // Standard deviations
        // Logger.recordOutput("Rebuilt/Vision/Config/StdDev X", currentVisionStdDevX);
        // Logger.recordOutput("Rebuilt/Vision/Config/StdDev Y", currentVisionStdDevY);
        // Logger.recordOutput("Rebuilt/Vision/Config/StdDev Theta", currentVisionStdDevTheta);

        // Logger.recordOutput("Rebuilt/Vision/Operator Overriden Vision Trust", operatorForcedVisionTrust);
        
        // Fusion thresholds
        // Logger.recordOutput("Rebuilt/Vision/Config/Max Translation Diff", maxTranslationDiff);
        // Logger.recordOutput("Rebuilt/Vision/Config/Max Rotation Diff", Math.toDegrees(maxRotationDiff));
        // Logger.recordOutput("Rebuilt/Vision/Distance From target", getDistanceToTarget(getTargetRelativeToBot()));

        // for (LimelightObject limelight : LIMELIGHT_LIST) {
        //     String llName = limelight.getName();
        //     IMUData llIMUData = LimelightHelpers.getIMUData(llName);
        //     Logger.recordOutput("Rebuilt/Vision/" + llName + "/IMU Data/Robot Yaw", llIMUData.robotYaw);
        //     Logger.recordOutput("Rebuilt/Vision/" + llName + "/IMU Data/Raw Yaw", llIMUData.Yaw);
        //     Logger.recordOutput("Rebuilt/Vision/" + llName + "/IMU Data/Roll", llIMUData.Roll);
        //     Logger.recordOutput("Rebuilt/Vision/" + llName + "/IMU Data/Pitch", llIMUData.Pitch);
        // }

        if (logDebuggingToggle.get()) {
            // Per-camera data
            for (LimelightObject limelight : LIMELIGHT_LIST) {
                String name = limelight.getName();
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
                
                if (results != null) {
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Valid", results.valid);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Priority Tag", LimelightHelpers.getFiducialID(name));
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Pipeline Latency", results.latency_pipeline);
                    Logger.recordOutput("Rebuilt/Vision/" + name + "/Capture Latency", results.latency_capture);
                    
                    // Get botpose data if available
                    double[] botpose = LimelightHelpers.getBotPose_wpiBlue(name);
                    if (botpose != null && botpose.length >= 6) {
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/X", botpose[0]);
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Y", botpose[1]);
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Z", botpose[2]);
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Roll", botpose[3]);
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Pitch", botpose[4]);
                        Logger.recordOutput("Rebuilt/Vision/" + name + "/Botpose/Yaw", botpose[5]);
                    }
                    
                    // Ambiguity for this camera
                    // Logger.recordOutput("Rebuilt/Vision/" + name + "/Ambiguity", computeAmbiguity(name));
                }
            }
            
            // Robot state
            Logger.recordOutput("Rebuilt/Vision/Robot/Pigeon Yaw", 
                drivetrain.getPigeon2().getYaw().getValueAsDouble());

            // Current bot pose
            // Logger.recordOutput("Rebuilt/Vision/CurrentPose", drivetrain.getState().Pose);
            Logger.recordOutput("Rebuilt/Vision/CurrentPose", drivetrain.getState().Pose);
            // getAdjustedFutureTarget(.5);

        }

        // Fault detection
        Logger.recordOutput("Rebuilt/Vision/Faults/Drivetrain instance null?", drivetrain == null);
        Logger.recordOutput("Rebuilt/Vision/Faults/Limelight List Empty?", LIMELIGHT_LIST.isEmpty());
        Logger.recordOutput("Rebuilt/Vision/Faults/Limelight Count", LIMELIGHT_LIST.size());
    }

    // -------------------------
    // Public API
    // -------------------------
    public Pose2d getLastFusedPose() { return lastFusedPose; }
    public Pose2d getLastVisionPose() { return lastVisionPose; }

    /**
     * Updates vision measurement standard deviations for fusion/filtering.
     */
    public void setVisionMeasurementStdDevs(double xMeters, double yMeters, double thetaRadians) {
        setVisionMeasurementStdDevs(VecBuilder.fill(defaultVisionStdDevX, defaultVisionStdDevY, defaultVisionStdDevTheta));
    }

    public void setVisionMeasurementStdDevs(Vector<N3> vector) {
        this.currentVisionStdDevX = vector.get(0);
        this.currentVisionStdDevY = vector.get(1);
        this.currentVisionStdDevTheta = vector.get(2);

        drivetrain.setVisionMeasurementStdDevs(vector);
    }

    public void setThrottle(int framesToSkip) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetThrottle(limelight.getName(), framesToSkip);
        }
    }

    public void setLLIMUModes(int mode) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetIMUMode(limelight.getName(), mode); // Use internal IMU + external IMU
        }
    }

    public void setLLIMUAssistAlpha(double alphaValue) {
        for (LimelightObject limelight : LIMELIGHT_LIST) {
            LimelightHelpers.SetIMUAssistAlpha(limelight.getName(), alphaValue);
        }
    }

    public void setMaxDiffForFusion(double translationMeters, double rotationRadians) {
        this.maxTranslationDiff = translationMeters;
        this.maxRotationDiff = rotationRadians;
    }

    @SuppressWarnings("static-access")
    public void setDrivetrainInstance(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Command setOperatorOverrideVisionTrustCommand(boolean value) {
        return new InstantCommand(() -> operatorForcedVisionTrust = value);
    }

    // -------------------------
    // Inner classes
    // -------------------------

    /**
     * Wrapper class for vision measurements.
     * Contains pose, confidence, timestamp, and validity flag.
     */
    private record VisionMeasurement(Pose2d pose, double confidence, double timestamp, boolean valid) {
        static VisionMeasurement invalid() {
            return new VisionMeasurement(null, 0.0, 0.0, false);
        }

        boolean isValid() {
            return valid && pose != null;
        }
    }

    private record limelightNetworkTableData(String name, 
                                             boolean hasATarget, 
                                             PoseEstimate megaTag2Estimate,
                                             PoseEstimate megaTag1Estimate,
                                             double[] botPose, 
                                             RawFiducial[] fiducials,
                                             IMUData imuData
                                            ) {}
}