// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final class RobotConstants {

    public static final String DRIVEBASE_CANBUS_NAME = "CantDrive";
    public static final String MAIN_SYSTEMS_CANBUS_NAME = "CantShoot";
    public static final String RIO_BUS_NAME = "Rio";

    public static final CANBus DRIVEBASE_CANBUS = new CANBus(DRIVEBASE_CANBUS_NAME);
    public static final CANBus MAIN_SYSTEMS_CANBUS = new CANBus(MAIN_SYSTEMS_CANBUS_NAME);
    public static final CANBus RIO_BUS = new CANBus(RIO_BUS_NAME);

    public static final boolean ENABLE_FOC = true;

    public static final NeutralModeValue DEFAULT_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    public static final class LEDConstants {
      public static final class DeviceIDs {
        public static final int kCANDLEID = 45;
      }
    }

    public static final class ClimberConstants {

      public static final class DeviceIDs {
        public static final int kElevator = 40;
        public static final int kTelescope = 41;
      }

      public static final class ClimbPositions {
        public static final double kElevatorReach = -1;
        public static final double kElevatorHandoff = -0.5;
        public static final double kElevatorL3Store = -120; //TODO: make lower, thinking maybe -45 or -50
        public static final double kElevatorClimb = -141.0; //Old: -146.5Mech Max: -148
        public static final double kElevatorStore = -55.4;

        public static final double kTelescopeStore = 0.5;
        public static final double kTelescopeHandoff = 4.5;
        public static final double kTelescopeL1Reach = 29.5;
        public static final double kTelescopeL2AndL3Reach = 29.5;
        public static final double kTelescopeAutoClimb = 4.5;
      }

    }

    public static final class IntakeConstants {

      public static final double kAngleEncoderMagnetOffset = -0.2001953125;
      public static final double kAngleRotorToSensorRatio = 36;//36:1

      public static final class DeviceIDs {
        public static final int kAngleMotorID = 20;
        public static final int kWheelMotorID = 21;
        public static final int kWheelMotorIDFollower = 22;
        public static final int kAngleEncoderID = 23;
      }

    }

    public static final class ShooterConstants {

      public static final class DeviceIDs {
        public static final int kAngleMotorID = 35;
        public static final int kLeftShooterMotorID = 36;
        public static final int kRightShooterMotorID = 37;
      }

      public static final class HoodConstants {
        public static final double degreesPerMotorRev = 2.2040816372; //2.0571428571429;
      }

    }

    public static final class TransferConstants {

      public static final class DeviceIDs {
        public static final int kDrivingMotorID = 25;
      }

    }

    public static final class TurretConstants {

      public static final class DeviceIDs {
        public static final int kMotorID = 30;
        public static final int kAbsEncoderID = 31;
      }

      //7in off from center
      public static final Transform2d OFFSET_WRT_CENTER_OF_ROBOT = new Transform2d(new Translation2d(-0.1443, 0), new Rotation2d(0)); // .1778
      public static final double ENCODER_MAGNET_OFFSET = -0.2685546875;

      //To find the ratios, divide if reduction, multiply if increase.
      //Example, if all reductions, and 7.5 -> 7 -> 7 -> 7.5: 7.5/7/7/7.5
      public static final double ROTOR_TO_SENSOR_RATIO = 0.022211653247;//01632653; //42.552137677193921189;// 0.02350058198; //1 cycoidal, 1 7.5:1: 0.0625850 //Both Cycoidals: 0.001827990359168
      public static final double SENSOR_TO_MECHANISM_RATIO = 0.93288943638;// Non-Lantern: 0.4565217391 * 1.06; Cycoildal-Lanter: 0.311279 * 0.9849444

      public static final double TURRET_OFFSET_TO_ZERO = 180;
      public static final double TURRET_OFFSET_TO_TARGET = 0.25; // 1.25

      public static final double TOTAL_ROTATION_DEGREES = 320; //320
      public static final double TOTAL_ROTATION_RADIANS = Math.toRadians(TOTAL_ROTATION_DEGREES);
      
      public static final double DEGREE_CHANGE_PER_MOTOR_REVOLUTION = 8.5714285714; //2.24;
      public static final double ABS_ENCDR_POS_TO_ANGLE = 360;
    }
    
    public static final class LimelightConstants {

      public static final class FrontLeft {
        public static final String NAME = "limelight-fl";
        public static final double LOCATION_X = 0.271457252;
        public static final double LOCATION_Y = -0.26510724;
        public static final double LOCATION_Z = 0.22526;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = 45;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class FrontRight {
        public static final String NAME = "limelight-fr";
        public static final double LOCATION_X = 0.271457252;
        public static final double LOCATION_Y = 0.26510724;
        public static final double LOCATION_Z = 0.22526;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = -45;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class BackLeft {
        public static final String NAME = "limelight-bl";
        public static final double LOCATION_X = -0.271457252;
        public static final double LOCATION_Y = -0.26510724;
        public static final double LOCATION_Z = 0.22526;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = 135;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class BackRight {
        public static final String NAME = "limelight-br";
        public static final double LOCATION_X = -0.271457252;
        public static final double LOCATION_Y = 0.26510724;
        public static final double LOCATION_Z = 0.22526;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = -135;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

    }

  }

  public static enum AlignPos {
    LEFT,
    RIGHT
  }

  public static final class FieldConstants {

    public static boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    public static AprilTagFieldLayout FIELD_LAYOUT;

    static {
      FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    /**
     * Ferry Poses are from blue alliance driver box perspective
     */
    public static final class ImportantFieldPoses {
      public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256, 4.0345);
      public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154, 4.0345);

      public static final Translation2d FERRY_TOP_TARGET = new Translation2d(2.54,7.0); //  new Translation2d(2.54, 6.1976); // Used to be .455, 6.95, 2.112, 6.432
      public static final Translation2d FERRY_BOTTOM_TARGET = new Translation2d(2.54,1.0); //(2.54, 1.8), (2.54, 6.1976)
      public static final Translation2d NEUTRAL_TARGET_BOTTOM = new Translation2d(2.396,  4.041);
      public static final Translation2d NEUTRAL_TARGET_TOP = new Translation2d(2.396,  4.041);
      public static final Translation2d FERRY_MID_RIGHT_TARGET = new Translation2d(3.418,  .5);
      public static final Translation2d FERRY_MID_LEFT_TARGET = new Translation2d(3.418,  7.6);
      public static final Translation2d FERRY_MID_TARGET = new Translation2d(2.446, 4.046);


      /* ========= HARD CODED POSES ======== */
      public static final Translation2d BLUE_LEFT_FERRY_TARGET = new Translation2d();
      public static final Translation2d BLUE_RIGHT_FERRY_TARGET = new Translation2d();

      public static final Translation2d RED_LEFT_FERRY_TARGET = new Translation2d();
      public static final Translation2d RED_RIGHT_FERRY_TARGET = new Translation2d();

      public static final Translation2d BLUE_TOWER = new Translation2d(1.063, 3.75);
      public static final Translation2d RED_TOWER = new Translation2d(15.475, 4.325);

    }

    public static final class AprilTagIDs {
      public static final int RED_BOTTOM_TRENCH_NEUTRAL_SIDE = 1;
      public static final int RED_BOTTOM_HUB_NEUTRAL_SIDE = 2;
      public static final int RED_NEUTRAL_HUB_SIDE_LOWER = 3;
      public static final int RED_NEUTRAL_HUB_SIDE_HIGHER = 4;
      public static final int RED_TOP_HUB_NEUTRAL_SIDE = 5;
      public static final int RED_TOP_TRENCH_NEUTRAL_SIDE = 6;
      public static final int RED_TOP_TRENCH_ALLIANCE_SIDE = 7;
      public static final int RED_TOP_HUB_ALLIANCE_SIDE = 8;
      public static final int RED_ALLIANCE_HUB_SIDE_HIGHER = 9;
      public static final int RED_ALLIANCE_HUB_SIDE_LOWER = 10;
      public static final int RED_BOTTOM_HUB_ALLIANCE_SIDE = 11;
      public static final int RED_BOTTOM_TRENCH_ALLIANCE_SIDE = 12;
      public static final int RED_OUTPOST_LOWER = 13;
      public static final int RED_OUTPOST_HIGHER = 14;
      public static final int RED_TOWER_LOWER = 15;
      public static final int RED_TOWER_HIGHER = 16;

      public static final int BLUE_TOP_TRENCH_NEUTRAL_SIDE = 17;
      public static final int BLUE_TOP_HUB_NEUTRAL_SIDE = 18;
      public static final int BLUE_NEUTRAL_HUB_SIDE_HIGHER = 19;
      public static final int BLUE_NEUTRAL_HUB_SIDE_LOWER = 20;
      public static final int BLUE_BOTTOM_HUB_NEUTRAL_SIDE = 21;
      public static final int BLUE_BOTTOM_TRENCH_NEUTRAL_SIDE = 22;
      public static final int BLUE_BOTTOM_TRENCH_ALLIANCE_SIDE = 23;
      public static final int BLUE_BOTTOM_HUB_ALLIANCE_SIDE = 24;
      public static final int BLUE_ALLIANCE_HUB_SIDE_LOWER = 25;
      public static final int BLUE_ALLIANCE_HUB_SIDE_HIGHER = 26;
      public static final int BLUE_TOP_HUB_ALLIANCE_SIDE = 27;
      public static final int BLUE_TOP_TRENCH_ALLIANCE_SIDE = 28;
      public static final int BLUE_OUTPOST_HIGHER = 29;
      public static final int BLUE_OUTPOST_LOWER = 30;
      public static final int BLUE_TOWER_HIGHER = 31;
      public static final int BLUE_TOWER_LOWER = 32;
    }

    public static Pose3d getTag3dPose(int tagID) {
      return FIELD_LAYOUT.getTagPose(tagID).orElseThrow(() -> {
        final String errorMsg = String.format("getTagPose called for unexpected tag, %d", tagID);
        return new RuntimeException(errorMsg);
      });
    }

    /**BLUE                      RED
     * -----------------------------
     * |    1 | 2          5 | 6   |
     * |     HUB            HUB    |
     * |    3 | 4          7 | 8   |
     * -----------------------------
     */
    public static final class TrenchManeuvering {

      public static enum TrenchLocationWRTBlueOrigin {
        CLOSE_LEFT,
        CLOSE_RIGHT,
        FAR_LEFT,
        FAR_RIGHT
      }
      
      public static final Translation2d trenchOne = new Translation2d(2.618, 7.435);
      public static final Translation2d trenchTwo = new Translation2d(6.618, 7.435);
      public static final Translation2d trenchThree = new Translation2d(2.618, 0.627);
      public static final Translation2d trenchFour = new Translation2d(6.618, 0.627);

      public static final Translation2d trenchFive = new Translation2d(9.618, 7.435);
      public static final Translation2d trenchSix = new Translation2d(14.618, 7.435);
      public static final Translation2d trenchSeven = new Translation2d(9.618, 0.627);
      public static final Translation2d trenchEight = new Translation2d(14.618, 0.627);

    }

    public static final class ZoneConstants {

      public static final class ZoneConstantsWelded {
        public static final Translation2d RED_CORNER = new Translation2d(16.54, 8.069326);

      }
      public static final class ZoneTargets {
        public static final Translation2d SHOOTING_TARGET = ImportantFieldPoses.BLUE_HUB_CENTER;
        public static final Translation2d FERRY_TOP_TARGET = ImportantFieldPoses.FERRY_TOP_TARGET;
        public static final Translation2d FERRY_BOTTOM_TARGET = ImportantFieldPoses.FERRY_BOTTOM_TARGET;
        public static final Translation2d NEUTRAL_TOP_TARGET = ImportantFieldPoses.NEUTRAL_TARGET_TOP;
        public static final Translation2d NEUTRAL_BOTTOM_TARGET = ImportantFieldPoses.NEUTRAL_TARGET_BOTTOM;
        public static final Translation2d FERRY_MID_RIGHT_TARGET = ImportantFieldPoses.FERRY_MID_RIGHT_TARGET;
        public static final Translation2d FERRY_MID_LEFT_TARGET = ImportantFieldPoses.FERRY_MID_LEFT_TARGET;
        public static final Translation2d FERRY_MID_TARGET = ImportantFieldPoses.FERRY_MID_TARGET;
      }
    }

  }
}
