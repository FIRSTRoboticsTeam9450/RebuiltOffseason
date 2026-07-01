// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.util.LocalADStarAK;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static PowerDistribution PDH;
  private final Field2d field2d = new Field2d();

  // sendableBuilder used to send the field2d to the network table which Elastic will look at to get the feild 2d
  private NetworkTable networkTable;
  private final SendableBuilderImpl sendableBuilder = new SendableBuilderImpl();

  private double updateValuesOffset = 0;

  /*** 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    Logger.recordMetadata("Rebuilt", "main");
    Logger.addDataReceiver(new NT4Publisher());


    PDH = new PowerDistribution();
    m_robotContainer = new RobotContainer();

    Logger.start();

    RobotController.setBrownoutVoltage(6.8); //RoboRio 1: 6.3, Default: 6.8, DANGER, VERY SCARY, IS A VERY BIG NUCLEAR OPTION
    // SignalLogger.setPath("/home/lvuser/logs/");
  }

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // Creates a network table which is read by Elastic (Seems like advantage scope doesn't see it)
    networkTable = NetworkTableInstance.getDefault()
            .getTable("FieldTable") // Name of the folder Field2d is stored in
            .getSubTable("Field2d"); // Name of the Field2d
    
    // Makes the builder attached to the network table to send the feild2d
    sendableBuilder.setTable(networkTable); 

    // initiate the feild2d and builder
    field2d.initSendable(sendableBuilder);
    sendableBuilder.startListeners();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //  Puts the robots pose2d into the feild and add it to the builder
    field2d.setRobotPose(RobotContainer.drivetrain.getState().Pose);
    sendableBuilder.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    FieldConstants.isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().disabledStdDev);
    VisionSubsystem.getInstance().setThrottle(100);
    VisionSubsystem.getInstance().setLLIMUModes(1);
    updateValuesOffset = 0;
    IntakeSubsystem.getInstance().init();
    LEDSubsystem.getInstance().setState(LEDState.TEAM_UNIQUE_DISABLED);
    // SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {
    if (updateValuesOffset >= 60) {
      updateValuesOffset = 0;
      TurretSubsystem.getInstance().updateTunableParameters();
    } else if (updateValuesOffset == 45) {
      ShooterSubsystem.getInstance().updateShooterConstants();
    } else if (updateValuesOffset == 30) {
      IntakeSubsystem.getInstance().updateConstants();
    } else if (updateValuesOffset == 15) {
      // ClimberSubsystem.getInstance().updateClimberConstants();
    }
    updateValuesOffset++;
  }

  @Override
  public void disabledExit() {
    FieldConstants.isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    VisionSubsystem.getInstance().setThrottle(0);
    VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().enabledMT2StdDev);
    VisionSubsystem.getInstance().setLLIMUModes(0);
    // VisionSubsystem.getInstance().setLLIMUAssistAlpha(0.01);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    VisionSubsystem.getInstance().setVisionMeasurementStdDevs(VisionSubsystem.getInstance().enabledMT2StdDev);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    IntakeSubsystem.getInstance().init();
    RobotContainer.drivetrain.setStatorCurrentLimit(60);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CoordinationSubsystem.getInstance().setHardTrenchOverride(false);
    IntakeSubsystem.getInstance().init();
    RobotContainer.drivetrain.setStatorCurrentLimit(30);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    VisionSubsystem.getInstance().setUseCustomTarget(false);
    // SignalLogger.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
