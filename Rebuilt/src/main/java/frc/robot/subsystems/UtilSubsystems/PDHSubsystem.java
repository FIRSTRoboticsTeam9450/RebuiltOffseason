// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UtilSubsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDHSubsystem extends SubsystemBase {
  private static PDHSubsystem INSTANCE;

  // Defaults to CAN ID 0 for CTRE, 1 for REV (should auto-detect type and id)
  private final PowerDistribution PDH = new PowerDistribution();
  private final int CAN_ID = PDH.getModule();
  private final ModuleType TYPE = PDH.getType();
  private final PowerDistributionVersion VERSION_NUMBERS = PDH.getVersion();
  
  private final LoggedNetworkBoolean logDebuggingToggleMatch = new LoggedNetworkBoolean("Tuning/Debugging Toggles/PDH Match", true);
  private final LoggedNetworkBoolean logDebuggingToggleNonMatch = new LoggedNetworkBoolean("Tuning/Debugging Toggles/PDH Non-Match", false);
  private final LoggedNetworkBoolean logDebuggingToggleDetails = new LoggedNetworkBoolean("Tuning/Debugging Toggles/PDH Extra Details", false); // For less important values (like firmware versions)

  /** Creates a new PowerSubsystem. */
  public PDHSubsystem() {
  }
  public static PDHSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new PDHSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    if (logDebuggingToggleMatch.get()) {
      Logger.recordOutput("Rebuilt/PDH/BatteryVoltage", getBatteryVoltage());
    }
    if (logDebuggingToggleNonMatch.get()) {
      // Doesn't use the method to calculate total current draw because this reduces vendordep method calls (already getting all channels)
      double[] currents = getAllCurrentAmpPulls(); double currentsSum = 0.0;
      for (double current : currents) currentsSum += current;
      Logger.recordOutput("Rebuilt/PDH/AmpPulls", currents);
      Logger.recordOutput("Rebuilt/PDH/CalculatedTotalAmpPull", currentsSum);
    }
    if (logDebuggingToggleDetails.get()) {
      Logger.recordOutput("Rebuilt/PDH/Info/CanId", getCanId());
      Logger.recordOutput("Rebuilt/PDH/Info/IsREV", isREV());
      Logger.recordOutput("Rebuilt/PDH/Info/NumChannels", getNumChannels());

      PowerDistributionVersion version = getVersion();
      Logger.recordOutput("Rebuilt/PDH/Info/Version/FirmwareMajor", version.firmwareMajor);
      Logger.recordOutput("Rebuilt/PDH/Info/Version/FirmwareMinor", version.firmwareMinor);
      Logger.recordOutput("Rebuilt/PDH/Info/Version/FirmwareFix", version.firmwareFix);
      Logger.recordOutput("Rebuilt/PDH/Info/Version/HardwareMajor", version.hardwareMajor);
      Logger.recordOutput("Rebuilt/PDH/Info/Version/HardwareMinor", version.hardwareMinor);
      Logger.recordOutput("Rebuilt/PDH/Info/Version/UniqueId", version.uniqueId);
    }
  }

  public int getCanId() {
    return CAN_ID;
  }
  public PowerDistributionVersion getVersion() {
    return VERSION_NUMBERS;
  }
  public boolean isREV() {
    return TYPE == ModuleType.kRev;
  }
  public int getNumChannels() {
    if (isREV()) return 24;
    else return 16;
  }

  public double getBatteryVoltage() {
    return PDH.getVoltage();
  }
  /** Warning: The returned number is rounded to an integer. */
  public double getReportedTotalCurrentAmpPull() {
    return PDH.getTotalCurrent();
  }
  public double getCalculatedTotalCurrentAmpPull() {
    double[] currents = getAllCurrentAmpPulls(); double currentsSum = 0.0;
    for (double current : currents) currentsSum += current;
    return currentsSum;
  }

  /** REV: Channels 0-23, CTRE: Channels 0-15 */
  public double getCurrentAmpPull(int channel) {
    return PDH.getCurrent(channel);
  }
  /** REV: Channels 0-23, CTRE: Channels 0-15 */
  public double[] getAllCurrentAmpPulls() {
    return PDH.getAllCurrents();
  }

  /** CTRE only, always returns 0 for REV */
  public double getPowerTempCelsiusCTRE() {
    return PDH.getTemperature();
  }
  /** CTRE only, always returns 0 for REV */
  public double getTotalPowerUsedCTRE() {
    return PDH.getTotalPower();
  }
  /** CTRE only, always returns 0 for REV */
  public double getTotalEnergyUsedCTRE() {
    return PDH.getTotalEnergy();
  }
  /** CTRE only, does nothing for REV */
  public void resetTotalEnergyUsedCTRE() {
    PDH.resetTotalEnergy();
  }
}
