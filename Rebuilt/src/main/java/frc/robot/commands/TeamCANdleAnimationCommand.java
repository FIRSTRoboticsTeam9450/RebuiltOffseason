// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Displays a custom CANdle animation. 
 * <br><br/>
 * Unused due to not having enough weight to put LEDs on robot.
 * No longer unused :D
 */
public class TeamCANdleAnimationCommand extends Command {
  // Command data
  private LEDSubsystem m_leds;

  // Config - Options
  public static final double FRAMERATE = 6.0;
  private final double BASE_BRIGHTNESS = 1.0;

  // Config - Color data
  public static final RGBWColor BLUE              = new RGBWColor(  0,   5,  70);
  public static final RGBWColor BLUE_GRADIENT     = new RGBWColor(  0,  11, 135);
  public static final RGBWColor BLUE_GRADIENT_2   = new RGBWColor(  0,  15, 210);
  public static final RGBWColor ORANGE            = new RGBWColor(255,  17,   0);
  public static final RGBWColor ORANGE_GRADIENT   = new RGBWColor(220,  25,   0);
  public static final RGBWColor 
  ORANGE_GRADIENT_2 = new RGBWColor(225,  35,   0);

  // Color arrays
  private ArrayList<RGBWColor> COLORS = new ArrayList<>();
  private ArrayList<RGBWColor> COLORS_CANDLE = new ArrayList<>();

  // Vars
  private final Timer timer = new Timer();
  private boolean hasStarted = false;
  private int offset = 0;
  private int offsetCandle = 0;

  /** Creates a new TeamCANdleAnimation. */
  public TeamCANdleAnimationCommand(LEDSubsystem leds) {
    addRequirements(leds);
    m_leds = leds;

    // Config - Color arrays data

    COLORS.add(BLUE);
    COLORS.add(BLUE_GRADIENT);
    COLORS.add(BLUE_GRADIENT_2);
    COLORS.add(ORANGE_GRADIENT_2);
    COLORS.add(ORANGE_GRADIENT);
    COLORS.add(ORANGE);
    addReversedColorsToArray(COLORS);

    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);

    // Color arrays final setup
    applyBaseBrightnessToArray(COLORS);
    applyBaseBrightnessToArray(COLORS_CANDLE);
    reverseArray(COLORS);
    reverseArray(COLORS_CANDLE);
  }

  private void addReversedColorsToArray(ArrayList<RGBWColor> arr) { // a,b,c,d,e,f -> a,b,c,d,e,f,e,d,c,b
    int size = arr.size();
    for (int i = size - 2; i >= 1; i--) arr.add(arr.get(i));
  }

  private void applyBaseBrightnessToArray(ArrayList<RGBWColor> arr) {
    for (int i = 0; i < arr.size(); i++) arr.set(i, arr.get(i).scaleBrightness(BASE_BRIGHTNESS));
  }
  
  private void reverseArray(ArrayList<RGBWColor> arr) {
    for (int i = 0; i < arr.size() / 2; i++) {
      int endI = arr.size() - 1 - i;
      RGBWColor temp = arr.get(i);
      arr.set(i, arr.get(endI));
      arr.set(endI, temp);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leds.removeLEDColor(false);

    hasStarted = false;
    offset = 0;
    offsetCandle = 0;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(1.0 / FRAMERATE) || !hasStarted) {
      hasStarted = true;

      offset = animateSection(COLORS, offset, LEDSubsystem.START_INDEX_STRIPONLY, LEDSubsystem.END_INDEX);
      offsetCandle = animateSection(COLORS_CANDLE, offsetCandle, LEDSubsystem.START_INDEX, LEDSubsystem.END_INDEX_NOSTRIP);
    }
  }

  private int animateSection(ArrayList<RGBWColor> arr, int offset, int start, int end) { // Set return value into offset variable
    int ledOffset = arr.size() - ((end + 1 - start) % arr.size()); // Used to start the animation at the correct place
    for (int led = end; led >= start; led--) {
      int colorIndex = (offset + ledOffset) % arr.size();
      ledOffset++;

      m_leds.setColorRange(led, led);
      m_leds.setColor(arr.get(colorIndex));
      m_leds.applyColorControl();
    }
    return (offset + 1) % arr.size(); // Update offset
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_leds.removeLEDColor(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
