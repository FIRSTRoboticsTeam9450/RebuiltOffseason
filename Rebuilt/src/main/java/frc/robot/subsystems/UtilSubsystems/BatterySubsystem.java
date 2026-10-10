// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.UtilSubsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BattFuelGaugeFixed;
import frc.robot.util.LoggingUtils;

public class BatterySubsystem extends SubsystemBase {
  private static BatterySubsystem INSTANCE;

  private boolean LOG_ON_LOAD = false;

  public final BattFuelGaugeFixed BFG = new BattFuelGaugeFixed(80); // 80

  private final LoggedNetworkBoolean logDebuggingToggleMatch = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Battery Match", true);
  private final LoggedNetworkBoolean logDebuggingToggleNonMatch = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Battery Non-Match", false);
  private final LoggedNetworkBoolean logDebuggingToggleDetails = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Battery Extra Details", false); // For less important values (like firmware versions)

  /** Creates a new BatterySubsystem. */
  public BatterySubsystem() {
  }
  public static BatterySubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new BatterySubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    if (LOG_ON_LOAD) {
      BFG.saveLog("");
      LOG_ON_LOAD = false;
    }
    float effectiveCapacityAh = BFG.getEffectiveCapacityAh();
    float depthOfDischargeAh = BFG.getDodAh();
    float stateOfChargeAh = effectiveCapacityAh - depthOfDischargeAh; // Labeled as "remaining charge" in docs
    Logger.recordOutput("Rebuilt/BatterySubsystem/StateOfCharge_Ah", stateOfChargeAh);
    float stateOfChargePct = BFG.getRemainingChargePct();
    Logger.recordOutput("Rebuilt/BatterySubsystem/StateOfCharge_Pct", stateOfChargePct);

    // No indents = Not shown on BFG screen. One indent = Partially shown on BFG screen. Two indents = Shown on BFG screen. (From base indent level)
    if (logDebuggingToggleMatch.get()) { // TODO: Logs here (don't put very many)
    }
    if (logDebuggingToggleNonMatch.get()) {
      Logger.recordOutput("Rebuilt/Battery/Connected_Bool", BFG.isConnected());
          Logger.recordOutput("Rebuilt/Battery/Nickname_Str", BFG.getNickname());
          Logger.recordOutput("Rebuilt/Battery/Manufacturer_Enum", BFG.getManufacturer());
          float capacityAh = BFG.getCapacityAh();
          Logger.recordOutput("Rebuilt/Battery/Capacity_Ah", capacityAh);
          Logger.recordOutput("Rebuilt/Battery/RatedCapacity_Ah", BFG.getRatedCapacityAh());
          Logger.recordOutput("Rebuilt/Battery/Age_Days", BFG.getBatteryAgeDays()); // Not actually in days, don't know why. Returns 4.3e16
          Logger.recordOutput("Rebuilt/Battery/NumCycles_Num", BFG.getNumCycles());

          Logger.recordOutput("Rebuilt/BatterySubsystem/Voltage_V", BFG.getVoltage());
          Logger.recordOutput("Rebuilt/BatterySubsystem/Current_A", BFG.getCurrent());
        Logger.recordOutput("Rebuilt/BatterySubsystem/State_Enum", BFG.getBatteryState());
      //     float effectiveCapacityAh = BFG.getEffectiveCapacityAh();
      //     Logger.recordOutput("Rebuilt/BatterySubsystem/EffectiveCapacity_Ah", effectiveCapacityAh);
      //     float effectiveCapacityPct = effectiveCapacityAh / capacityAh * 100.0F;
      //     Logger.recordOutput("Rebuilt/BatterySubsystem/EffectiveCapacity_Pct", effectiveCapacityPct);
      // float depthOfDischargeAh = BFG.getDodAh();
      // Logger.recordOutput("Rebuilt/BatterySubsystem/DepthOfDischarge_Ah", depthOfDischargeAh);
      // Logger.recordOutput("Rebuilt/BatterySubsystem/DepthOfDischarge_Wh", BFG.getDodWh());
      //   float stateOfChargeAh = effectiveCapacityAh - depthOfDischargeAh; // Labeled as "remaining charge" in docs
      //   Logger.recordOutput("Rebuilt/BatterySubsystem/StateOfCharge_Ah", stateOfChargeAh);
      //   float stateOfChargePct = BFG.getRemainingChargePct();
      //   Logger.recordOutput("Rebuilt/BatterySubsystem/StateOfCharge_Pct", stateOfChargePct);


          Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MinVoltage_V", BFG.getCycleMinVoltage());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MaxVoltage_V", BFG.getCycleMaxVoltage());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MinCurrent_A", BFG.getCycleMinCurrent());
          Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MaxCurrent_A", BFG.getCycleMaxCurrent());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/StartingVoltage_V", BFG.getCycleStartingVoltage());
          Logger.recordOutput("Rebuilt/Battery/CurrentCycle/RmsDischargeCurrent_A", BFG.getCycleRmsDischargeCurrent());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MinDepthOfDischarge_Ah", BFG.getCycleMinDodAh());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MinDepthOfDischarge_Wh", BFG.getCycleMinDodWh()); // Returns the same value as the current cycle min depth of discharge in amp-hours, but shouldn't
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MaxDepthOfDischarge_Ah", BFG.getCycleMaxDodAh());
      Logger.recordOutput("Rebuilt/Battery/CurrentCycle/MaxDepthOfDischarge_Wh", BFG.getCycleMaxDodWh());
        LoggingUtils.logSecondsAsTime("Rebuilt/Battery/CurrentCycle/ChargeTime_Time", (int) BFG.getCycleChargeTime());
        LoggingUtils.logSecondsAsTime("Rebuilt/Battery/CurrentCycle/DischargeTime_Time", (int) BFG.getCycleDischargeTime());


          Logger.recordOutput("Rebuilt/Battery/LastMatch/MinVoltage_V", BFG.getMatchMinVoltage());
      Logger.recordOutput("Rebuilt/Battery/LastMatch/MaxVoltage_V", BFG.getMatchMaxVoltage());
      Logger.recordOutput("Rebuilt/Battery/LastMatch/MinCurrent_A", BFG.getMatchMinCurrent());
          Logger.recordOutput("Rebuilt/Battery/LastMatch/MaxCurrent_A", BFG.getMatchMaxCurrent());
          Logger.recordOutput("Rebuilt/Battery/LastMatch/StartingVoltage_V", BFG.getMatchStartingVoltage());
          Logger.recordOutput("Rebuilt/Battery/LastMatch/RmsDischargeCurrent_A", BFG.getMatchRmsDischargeCurrent());
      Logger.recordOutput("Rebuilt/Battery/LastMatch/MinDepthOfDischarge_Ah", BFG.getMatchMinDodAh());
      Logger.recordOutput("Rebuilt/Battery/LastMatch/MaxDepthOfDischarge_Ah", BFG.getMatchMaxDodAh());
      LoggingUtils.logSecondsAsTime("Rebuilt/Battery/LastMatch/ChargeTime_Time", (int) BFG.getMatchChargeTime());
      LoggingUtils.logSecondsAsTime("Rebuilt/Battery/LastMatch/DischargeTime_Time", (int) BFG.getMatchDischargeTime());
          LoggingUtils.logSecondsAsTime("Rebuilt/Battery/LastMatch/Duration_Time", (int) BFG.getMatchDuration()); // Labeled as "active time" on webpage
      Logger.recordOutput("Rebuilt/Battery/LastMatch/StartingCharge_Ah", BFG.getMatchStartingCharge()); // Webpage calls this "Starting depth of discharge"
      Logger.recordOutput("Rebuilt/Battery/LastMatch/StartingCharge_Wh", BFG.getMatchStartingEnergy()); // Webpage calls this "Starting depth of discharge"
      Logger.recordOutput("Rebuilt/Battery/LastMatch/EndingCharge_Ah", BFG.getMatchEndingCharge()); // Webpage calls this "Ending depth of discharge"
      Logger.recordOutput("Rebuilt/Battery/LastMatch/EndingCharge_Wh", BFG.getMatchEndingEnergy()); // Webpage calls this "Ending depth of discharge"
    }
    if (logDebuggingToggleDetails.get()) {
      Logger.recordOutput("Rebuilt/Battery/SerialNumber_Num", BFG.getSerialNumber());
      Logger.recordOutput("Rebuilt/Battery/FirmwareVersion_Num", BFG.getFirmwareVersion());
    }


    // Shown on BFG but cannot get in code (or some of these would instead require math that I do not currently know about):
      // Match:
        // Discharging charge (Ah)
        // Discharging charge (kJ)
      // Cycle / LastCycle:
        // Charging charge (Ah)
        // Charging charge (kJ)
        // Discharging charge (Ah)
        // Discharging charge (kJ)
      // Lifetime:
        // Min voltage (V)
        // Max discharging charge (Ah)
        // Max discharging charge (kJ)
  }
}
