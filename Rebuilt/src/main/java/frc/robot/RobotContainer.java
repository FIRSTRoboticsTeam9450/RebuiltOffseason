// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlignPos;
import frc.robot.commands.DriveBrakeCommand;
import frc.robot.commands.PositionAlignCommand;
import frc.robot.commands.ResetIMU;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UtilSubsystems.BatterySubsystem;
import frc.robot.subsystems.UtilSubsystems.PDHSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbLevel;
import frc.robot.subsystems.ClimberSubsystem.ClimbStates;
import frc.robot.util.BezierCurve;


public class RobotContainer {
  public final static BatterySubsystem battery = BatterySubsystem.getInstance();
  public final static PDHSubsystem power = PDHSubsystem.getInstance();
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static VisionSubsystem vision = VisionSubsystem.getInstance();
  public final static LEDSubsystem led = LEDSubsystem.getInstance();
  public final static CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
  public final static ClimberSubsystem climbSub = ClimberSubsystem.getInstance();
  public static TurretSubsystem turretSub = TurretSubsystem.getInstance();
  public static ShooterSubsystem shooterSub = ShooterSubsystem.getInstance();
  public static TransferSubsystem transferSub = TransferSubsystem.getInstance();
  public static IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();


  /**
   * Woojins Drive: 117.4, 0.054, 91.4, 0.76, 0.1, 0.01
   * Woojins Rotate: 120.1, 0.145, 92.2, 0.362, 0.035, 0.03
   */
  public static BezierCurve driveBezier = new BezierCurve("drive", 151.0, 0.137, 116, 0.646, 0.1, 0.01);
  public static BezierCurve rotateBezier = new BezierCurve("rotate", 164.8, 0.093, 100, 0.84, 0.035, 0.03);

  
  public static SlewRateLimiter driveXAccelLimiter = new SlewRateLimiter(5);
  public static SlewRateLimiter driveYAccelLimiter = new SlewRateLimiter(5);
  public static SlewRateLimiter rotationAccelLimiter = new SlewRateLimiter(8.0);

  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  public static boolean slewMode = false;
  private static final double scalarFactorForSpeed = 0.15;

  public static double maxSpeed = DEFAULT_MAX_SPEED * 0.2;
  public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE * 1;

  private final CommandXboxController DRIVER = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController OPERATOR = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(driveBezier.getDeadband())
        .withRotationalDeadband(rotateBezier.getDeadband())    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    TurretSubsystem.getInstance().setDriverController(DRIVER);

    configureBindings();

    registeredCommands();
    
    autoChooser = new LoggedDashboardChooser<>("Rebuilt/Auto Chooser");
    configureAutos();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Full Sweep", drivetrain.getAutoPath("P - Top Start", false));
    autoChooser.addOption("Left Half Sweep + Depot", drivetrain.getAutoPath("HST - FPSD", false));
    autoChooser.addOption("Left Half Sweep + Climb", drivetrain.getAutoPath("HST - FPSC", false));
    autoChooser.addOption("Right Push Half Sweep", drivetrain.getAutoPath("HSB - FPSC", false));
    autoChooser.addOption("Right Fery Half Sweep", drivetrain.getAutoPath("FO - 2 Sweeps", false));

    autoChooser.addOption("Hub Depot", drivetrain.getAutoPath("Depot", false)); 
    autoChooser.addOption("Hub Depot With Bump", drivetrain.getAutoPath("Depot with Bump", false)); 
    autoChooser.addOption("Bump Depot", drivetrain.getAutoPath("Bump Depot", false));
    autoChooser.addOption("LLDisruptor", drivetrain.getAutoPath("Left Lead Disruptor", false));

    autoChooser.addOption("Preload + Outpost + Climb", drivetrain.getAutoPath("Outpost", false));
    autoChooser.addOption("Preload +  Climb", drivetrain.getAutoPath("Preload Only", false));
    autoChooser.addOption("Preload + Return to Hub", drivetrain.getAutoPath("Preload + Return to Hub", false));
        
    autoChooser.addOption("Intake From Hub", drivetrain.getAutoPath("Sit", false));

    autoChooser.addOption("CMP - Long Ferry", drivetrain.getAutoPath("CMP - Long Ferry", false));

    // autoChooser.addOption("Climb Test", drivetrain.getAutoPath("Climb Only", false));
    // autoChooser.addOption("Spin Test", drivetrain.getAutoPath("Spin Testing", false));
    // autoChooser.addOption("Slip Test", drivetrain.getAutoPath("Slip Testing", false));
    // autoChooser.addOption("Drive Test", drivetrain.getAutoPath("Drive Straight", false));
    // autoChooser.addOption("Drive Test S", drivetrain.getAutoPath("Drive S", false));
    // autoChooser.addOption("Drive And Return", drivetrain.getAutoPath("Drive And Return", false));
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
              
                // double driveX = driveXAccelLimiter.calculate(-driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed);
                // double driveY = driveYAccelLimiter.calculate(-driveBezier.getOutput(DRIVER.getLeftX())  * maxSpeed);
                double driveX = DRIVER.getLeftY();
                double driveY = DRIVER.getLeftX();
                boolean xNegative = driveX < 0;
                boolean yNegative = driveY < 0;
                driveX = xNegative ? (Math.pow(driveX, 2) * -1.0) : Math.pow(driveX, 2);
                driveY = yNegative ? (Math.pow(driveY, 2) * -1.0) : Math.pow(driveY, 2);
                double driveRotate = rotationAccelLimiter.calculate(-rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate);
                double joyX = -DRIVER.getLeftX();
                double joyY = DRIVER.getLeftY();
                double speed = driveBezier.getOutput(Math.hypot(joyX, joyY)) * maxSpeed;
                double rotation = Math.atan(joyY/joyX);
                if(joyY < 0 && joyX < 0) {
                  rotation = Math.PI - rotation;
                }
                else if(joyY > 0 && joyX < 0) {
                  rotation = Math.PI - rotation;
                }
                else if(joyY < 0 && joyX > 0) {
                  rotation *= -1;
                }
                else if(joyY > 0 && joyX > 0){
                  rotation *= -1;
                }
                else if(joyY > 0 && joyX == 0) {
                  rotation = -Math.PI/2;
                }
                else if(joyY < 0 && joyX == 0) {
                  rotation = Math.PI/2;
                }
                else if(joyY == 0 && joyX < 0) {
                  rotation = Math.PI;
                } else if (joyY == 0 && joyX > 0) {
                  rotation = 0;
                }
                else if(joyY == 0 && joyX > 0) {
                  rotation = 0;
                }
                else if(joyY == 0 && joyX == 0) {
                  rotation = 0;
                }
                double xUsed = speed * Math.cos(rotation);
                double yUsed = speed * Math.sin(rotation);
                
                // double rotate = -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate;
                double rotate = DRIVER.getRightX();
                boolean xRotatePositive = rotate > 0;
                rotate = xRotatePositive ? (Math.pow(rotate, 2) * -1.0) * maxAngularRate : Math.pow(rotate, 2) * maxAngularRate;

                return drive.withVelocityX(slewMode ? driveX : yUsed) // Drive forward with negative Y (forward)
                    .withVelocityY(slewMode ? driveY : xUsed) // Drive left with negative X (left)
                    .withRotationalRate(slewMode ? driveRotate : rotate); // Drive counterclockwise with negative X (left)
                
                // return drive.withVelocityX(slewMode ? driveX : -driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed) // Drive forward with negative Y (forward)
                //     .withVelocityY(slewMode ? driveY : (-driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed)) // Drive left with negative X (left)
                //     .withRotationalRate(slewMode ? driveRotate : -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate); // Drive counterclockwise with negative X (left)
            }
            )
        );
        
    // DRIVER.leftBumper().whileTrue(
    //   new PositionAlignCommand(drivetrain, AlignPos.LEFT)
    // );
    // DRIVER.rightBumper().whileTrue(
    //   new PositionAlignCommand(drivetrain, AlignPos.RIGHT)
    // );

    // DRIVER.leftTrigger().onTrue(
    //     new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * scalarFactorForSpeed).alongWith(new InstantCommand(() -> slewMode = true))
    //     // new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED).alongWith(new InstantCommand(() -> slewMode = true))
    // ).onFalse(
    //   new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * 1).alongWith(new InstantCommand(() -> slewMode = false))
    // );

    DRIVER.leftTrigger().onTrue(
        CoordinationSubsystem.getInstance().setFastTransferCommand(true).andThen(
        new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * scalarFactorForSpeed).alongWith(new InstantCommand(() -> slewMode = false)))
        // new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED).alongWith(new InstantCommand(() -> slewMode = true))
    ).onFalse(
      CoordinationSubsystem.getInstance().setFastTransferCommand(false).andThen(
      new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * 1).alongWith(new InstantCommand(() -> slewMode = false)))
    );
    
    DRIVER.leftTrigger().onTrue(
        CoordinationSubsystem.getInstance().setFastTransferCommand(true).andThen(
        new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * scalarFactorForSpeed).alongWith(new InstantCommand(() -> slewMode = false)))
        // new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED).alongWith(new InstantCommand(() -> slewMode = true))
    ).onFalse(
      CoordinationSubsystem.getInstance().setFastTransferCommand(false).andThen(
      new InstantCommand(() -> maxSpeed = DEFAULT_MAX_SPEED * 1).alongWith(new InstantCommand(() -> slewMode = false)))
    );



    // DRIVER.rightTrigger().onTrue(
    //   CoordinationSubsystem.getInstance().setOverrideCommand(true)
    // ).onFalse(
    //   CoordinationSubsystem.getInstance().setOverrideCommand(false)
    // );
    
    DRIVER.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setRejectingCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setRejectingCommand(false)
    );

    DRIVER.y().onTrue(new ResetIMU(drivetrain));

    DRIVER.rightBumper().whileTrue(new DriveBrakeCommand(drivetrain));

    DRIVER.back().onTrue(
      new InstantCommand(() -> {IntakeSubsystem.getInstance().zeroEncoders(); ShooterSubsystem.getInstance().zeroEncoder();}).ignoringDisable(true)
    );

    /* ==================== OPERATOR KEYBINDS ==================== */

    OPERATOR.leftTrigger().onTrue(
      CoordinationSubsystem.getInstance().setRejectingCommand(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setRejectingCommand(false)
    );

    OPERATOR.rightTrigger().onTrue(
      CoordinationSubsystem.getInstance().setIntakePushing(true)
    ).onFalse(
      CoordinationSubsystem.getInstance().setIntakePushing(false)
    );

    OPERATOR.leftBumper().onTrue(
      CoordinationSubsystem.getInstance().setIntakeCommand().alongWith(CoordinationSubsystem.getInstance().setFireCommand())
    );

    OPERATOR.rightBumper().onTrue(
      CoordinationSubsystem.getInstance().setFireCommand(false).alongWith(CoordinationSubsystem.getInstance().setIntakeCommand(false))
    );
    
    OPERATOR.rightStick().onTrue(
      CoordinationSubsystem.getInstance().setForceTrenchCommand()
    );

    OPERATOR.x().whileTrue(CoordinationSubsystem.getInstance().increaseManualAdjust());
    OPERATOR.y().onTrue(CoordinationSubsystem.getInstance().resetManualAdjust());
    OPERATOR.b().whileTrue(CoordinationSubsystem.getInstance().decreaseManualAdjust());

    OPERATOR.a().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setForceClimberDownCommand()).alongWith(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.PREPARING_FOR_SHOOTING, true)));

    OPERATOR.b().and(OPERATOR.x()).and(OPERATOR.y()).onTrue(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.INIT, true));

    OPERATOR.leftStick().onTrue(CoordinationSubsystem.getInstance().setShiftingCommand(false)).onFalse(CoordinationSubsystem.getInstance().setShiftingCommand(true));

    OPERATOR.povUp().whileTrue(
      new InstantCommand(() -> ShooterSubsystem.getInstance().addManualHoodShift())
    );
    OPERATOR.povDown().whileTrue(
      new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualHoodShift())
    );

    // OPERATOR.povUp().onTrue(
    //   Commands.either(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.STORING)), ClimberSubsystem.getInstance().setClimbStateAndLevelCommand(), (() -> DriverStation.isTest()))
    // );

    // OPERATOR.povDown().onTrue(
    //   VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(true)
    // ).onFalse(
    //   VisionSubsystem.getInstance().setOperatorOverrideVisionTrustCommand(false)
    // );

    // OPERATOR.povRight().onTrue(coordSub.setStateCommand(AbsoluteStates.EMERGENCY, true));
    // OPERATOR.povLeft().onTrue(coordSub.setStateCommand(AbsoluteStates.DEFENSE, true));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L1)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.INIT))));
    // OPERATOR.povLeft().onTrue(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB, false).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new WaitCommand(0.15)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT)))));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.END))));
    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> VisionSubsystem.getInstance().setFerryTarget("top")));
    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.HANDOFF))));

    OPERATOR.back().onTrue(ClimberSubsystem.getInstance().setForceClimberUpCommand());
    OPERATOR.start().onTrue(ClimberSubsystem.getInstance().setForceClimberDownCommand());

    /* ================ TEST ONLY KEYBINDS ============== */
    
    /* ============================== SYSID ==================================== */
    // DRIVER.y().whileTrue(shooterSub.sysIdQuasistatic(Direction.kForward));
    // DRIVER.a().whileTrue(shooterSub.sysIdQuasistatic(Direction.kReverse));
    // DRIVER.b().whileTrue(shooterSub.sysIdDynamic(Direction.kForward));
    // DRIVER.x().whileTrue(shooterSub.sysIdDynamic(Direction.kReverse));
    // DRIVER.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // DRIVER.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // DRIVER.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // DRIVER.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    
      
    /* =============================== DISABLED KEYBINDS ======================== */

    // DRIVER.x().onTrue(new InstantCommand(() ->VisionSubsystem.getInstance().setFerryTarget(2)));
    // DRIVER.y().onTrue(new InstantCommand(() ->VisionSubsystem.getInstance().setFerryTarget(0)));
    // DRIVER.b().onTrue(new InstantCommand(() ->VisionSubsystem.getInstance().setFerryTarget(1)));
    
    // DRIVER.povDown().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.CLOSE_LEFT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povLeft().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.FAR_LEFT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povRight().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.CLOSE_RIGHT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );
    // DRIVER.povUp().whileTrue(
    //     CoordinationSubsystem.getInstance().setTenchManeuveringKeybind(TrenchLocationWRTBlueOrigin.FAR_RIGHT)
    //   ).onFalse(
    //     new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //   );

    
    // OPERATOR.povLeft().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.INIT))));
    // OPERATOR.povUp().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.HANDOFF))));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.L2)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.END))));

    
    // OPERATOR.b().onTrue(climbSub.GoToNextLevel());
    // OPERATOR.povLeft().onTrue(CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB));
    // OPERATOR.povRight().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.INIT))));
    // OPERATOR.povDown().onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setClimbState(ClimbStates.END)));

  }


  public ChassisSpeeds getRobotFieldCentricVeloc() {
    return new ChassisSpeeds(-driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed, -driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed, -rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate);
  }


  public void registeredCommands() {
    // Intake Commands
    // NamedCommands.registerCommand("ZeroIntake", IntakeSubsystem.getInstance().setPosAndStateCommand(IntakePos.ZEROING, IntakeState.IDLING));
    NamedCommands.registerCommand("ZeroIntake", new InstantCommand());
    NamedCommands.registerCommand("IntakePush", CoordinationSubsystem.getInstance().setIntakePushing());

    NamedCommands.registerCommand("SetIntake", CoordinationSubsystem.getInstance().setIntakeCommand());
    NamedCommands.registerCommand("SetFire", CoordinationSubsystem.getInstance().setFireCommand());
    NamedCommands.registerCommand("SetFastTransfer", CoordinationSubsystem.getInstance().setFastTransferCommand());
    NamedCommands.registerCommand("SetHardTrenchOverrideTrue", CoordinationSubsystem.getInstance().setHardTrenchOverride(true));
    NamedCommands.registerCommand("SetHardTrenchOverrideFalse", CoordinationSubsystem.getInstance().setHardTrenchOverride(false));

    NamedCommands.registerCommand("SetFerryTargetBottom", new InstantCommand(() -> VisionSubsystem.getInstance().setFerryTarget("bottom")));
    NamedCommands.registerCommand("SetFerryTargetTop", new InstantCommand(() -> VisionSubsystem.getInstance().setFerryTarget("top")));

    NamedCommands.registerCommand("LowerShooterPower", new InstantCommand(() -> ShooterSubsystem.getInstance().subtractManualVelocityShift()));
    NamedCommands.registerCommand("IncreaseShooterPower", new InstantCommand(() -> ShooterSubsystem.getInstance().addManualVelocityShift(.25)));
    
    NamedCommands.registerCommand("LeftTurretAngle", new InstantCommand(() -> TurretSubsystem.getInstance().increaseManualAdjust()));
    NamedCommands.registerCommand("RightTurretAngle", new InstantCommand(() -> TurretSubsystem.getInstance().decreaseManualAdjust()));

    NamedCommands.registerCommand("ClimberUp", CoordinationSubsystem.getInstance().setStateCommand(AbsoluteStates.CLIMB, false).alongWith(new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new WaitCommand(0.15)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.INIT)))));
    NamedCommands.registerCommand("ClimberDown", new InstantCommand(() -> climbSub.setClimbLevel(ClimbLevel.AUTO)).alongWith(new InstantCommand(() -> climbSub.setClimbState(ClimbStates.HANDOFF))));
    // NamedCommands.registerCommand("ClimberDown", new InstantCommand());

    NamedCommands.registerCommand("ForceTrench", CoordinationSubsystem.getInstance().setForceTrenchCommand());
    NamedCommands.registerCommand("UseCustomTarget", VisionSubsystem.getInstance().setUseCustomTarget());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
