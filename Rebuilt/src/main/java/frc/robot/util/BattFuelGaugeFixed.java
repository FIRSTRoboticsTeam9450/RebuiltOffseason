// This file is here because the original file has a few bugs. There also seem to be a few bugs in the JNI.
package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import com.playingwithfusion.jni.BattFuelGaugeJNI;

/**
 * CAN based Battery Fuel Gauge (BFG) sensor instance
 * FIXED: Small formatting changes
 * 
 */
public class BattFuelGaugeFixed implements Sendable, AutoCloseable {
  /** FIXED: Made public */
  public final long m_handle;

  /**
   * Battery manufacturer enumeration
   */
  public enum BatteryManufacturer {
    Duracell,
    Energizer,
    Interstate,
    MightyMax,
    MKPowered,
    PowerSonic,
    NumMfgs
  }

  /**
   * Battery charge state enumeration
   */
  public enum BatteryChargeState {
    /**
     * Unknown charge/discharge state
     */
    Init,

    /**
     * Battery finished charging and has started discharging
     */
    Discharge,

    /**
     * Constant current charging
     */
    ConstICharge,

    /**
     * Constant voltage charging
     */
    ConstVCharge,

    /**
     * Trickle (top-off) charging
     */
    TrickleCharge,

    /**
     * Charging complete
     */
    Idle,

    /**
     * Open circuit (no-load connected to battery).  State of charge based on open circuit voltage.
     */
    MeasureOCV
  }


  /**
   * Create an instance of the CAN Battery Fuel Gauge sensor.
   *
   * This is designed to support the Playing With Fusion (PWF) ROB-70001 battery
   * fuel gauge sensor
   *
   * @param sensorId The 6-bit identifier used to select a particular
   *                 sensor on the CAN bus.  This identifier may be set
   *                 through the PWF Device configuration page on the 
   *                 roboRIO.
   */
  public BattFuelGaugeFixed(int sensorId) {
    m_handle = BattFuelGaugeJNI.create(sensorId);
    SendableRegistry.setName(this, "Battery Fuel Gauge", sensorId);
  }

  /**
   * Destroy the BattFuelGaugeFixed object and free any asscioated resources
   */
  @Override
  public void close() {
    if (m_handle != 0) BattFuelGaugeJNI.destroy(m_handle);
  }

  /* 
  * Initialize dashboard
  */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Battery Fuel Gauge");
    builder.addDoubleProperty( "Voltage",            this::getVoltage,                  null);
    builder.addDoubleProperty( "Current",            this::getCurrent,                  null);
    builder.addDoubleProperty( "DOD",                this::getDodAh,                    null);
    builder.addDoubleProperty( "Capacity",           this::getCapacityAh,               null);
    builder.addDoubleProperty( "Effective Capacity", this::getEffectiveCapacityAh,      null);
    builder.addDoubleProperty( "Charge Remaining",   this::getRemainingChargePct,       null);
    builder.addDoubleProperty( "Cycle RMS Current",  this::getCycleRmsDischargeCurrent, null);
    builder.addDoubleProperty( "Match Min Voltage",  this::getMatchMinVoltage,          null);
    builder.addBooleanProperty("Connected",          this::isConnected,                 null);
  }

  /**
   * Dispay 'Identification' page on the BFG OLED display
   * to indicate the sensor asscioated with this instance
   * of software
   */
  public void identifySensor() {
    BattFuelGaugeJNI.identify(m_handle);
  }

  /**
   * Determine the sensor firmware version
   * 
   * @return The sensor firmware version
   */
  public long getFirmwareVersion() {
    return BattFuelGaugeJNI.getFirmwareVersion(m_handle);
  }

  /**
   * Determine the sensor hardware serial number
   * 
   * @return The sensor hardware serial number
   */
  public long getSerialNumber() {
    return BattFuelGaugeJNI.getSerialNumber(m_handle);
  }

  /**
   * Saves the BFG state to a log file.
   *
   * @param filename Filename and path to write BFG state JSON file.  Use empty string
   *                 to use automatic file name in the /u/logs directory of a USB flash
   *                 drive or the /home/lvuser/logs directory in RoboRIO internal storage.
   *                 Automatic filenames are of the form: BFGSnapshot_{YYYMMDD}_{HH:MM:SS}_{Event Name}_{Match Type}{Match Number}.json, 
   *                 where the event name and Match information are derived from the FMS
   */
  public void saveLog(String filename) {
    BattFuelGaugeJNI.saveLog(m_handle, filename);
  }

  // -------------------------------------------------------------------------
  // Core real-time / state values
  // -------------------------------------------------------------------------

  /**
   * Determine the raw charge/discharge state of the battery
   * FIXED: Added this method
   * 
   * @return The raw charge/discharge state enumeration of the battery
   */
  public int getRawBatteryState() {
    return BattFuelGaugeJNI.getBatteryState(m_handle);
  }

  /**
   * Determine the charge/discharge state of the battery
   * FIXED: Returned state was offset down by one
   * 
   * @return The charge/discharge state enumeration of the battery
   */
  public BatteryChargeState getBatteryState() {
    BatteryChargeState state;
    int rawState = getRawBatteryState();

    if (rawState == 0) state = BatteryChargeState.Init;
    else if (rawState == 1) state = BatteryChargeState.Discharge;
    else if (rawState == 2) state = BatteryChargeState.ConstICharge;
    else if (rawState == 3) state = BatteryChargeState.ConstVCharge;
    else if (rawState == 4) state = BatteryChargeState.TrickleCharge;
    else if (rawState == 5) state = BatteryChargeState.Idle;
    else state = BatteryChargeState.MeasureOCV;

    return state;
  }

  /**
   * Gets the current battery discharge current.
   *
   * @return battery discharge current in Amps (A)
   */
  public float getCurrent() {
    return BattFuelGaugeJNI.getCurrent(m_handle);
  }

  /**
   * Gets the current battery terminal voltage.
   *
   * @return battery voltage in Volts (V)
   */
  public float getVoltage() {
    return BattFuelGaugeJNI.getVoltage(m_handle);
  }

  /**
   * Gets the current depth of discharge in ampere-hours.
   *
   * @return depth of discharge in Ah
   */
  public float getDodAh() {
    return BattFuelGaugeJNI.getDodAh(m_handle);
  }

  /**
   * Gets the current depth of discharge energy.
   *
   * @return depth of discharge energy in Watt-hours (Wh)
   */
  public float getDodWh() {
    return BattFuelGaugeJNI.getDodWh(m_handle);
  }

  /**
   * Gets the estimated remaining charge as a percentage.
   *
   * @return remaining charge in percent (typically 0-100)
   */
  public float getRemainingChargePct() {
    return BattFuelGaugeJNI.getRemainingChargePct(m_handle);
  }

  /**
   * Gets the effective (usable) capacity of the battery.
   *
   * @return effective capacity in Ah
   */
  public float getEffectiveCapacityAh() {
    return BattFuelGaugeJNI.getEffectiveCapacityAh(m_handle);
  }

  // -------------------------------------------------------------------------
  // Battery age, capacity & cycle information
  // -------------------------------------------------------------------------

  /**
   * Gets the age of the battery.
   * FIXED: Return type
   *
   * @return battery age in days
   */
  public long getBatteryAgeDays() {
    return BattFuelGaugeJNI.getBatteryAgeDays(m_handle);
  }

  /**
   * Gets the current estimated battery capacity.
   *
   * @return battery capacity in Ah
   */
  public float getCapacityAh() {
    return BattFuelGaugeJNI.getCapacityAh(m_handle);
  }

  /**
   * Gets the original rated capacity of the battery when new.
   *
   * @return rated capacity in Ah
   */
  public float getRatedCapacityAh() {
    return BattFuelGaugeJNI.getRatedCapacityAh(m_handle);
  }

  /**
   * Gets the total number of charge/discharge cycles experienced by the battery.
   *
   * @return number of charge/discharge cycles
   */
  public int getNumCycles() {
    return BattFuelGaugeJNI.getNumCycles(m_handle);
  }

  // -------------------------------------------------------------------------
  // Identification
  // -------------------------------------------------------------------------

  /**
   * Gets the user-defined nickname of this battery.
   *
   * @return battery nickname, or empty string if not set or unavailable
   */
  public String getNickname() {
    return BattFuelGaugeJNI.getNickname(m_handle);
  }

  /**
   * Gets the raw manufacturer identifier of the battery.
   * FIXED: Added this method
   *
   * @return Raw battery manufacturer reported by the BFG
   */
  public int getRawManufacturer() {
    return BattFuelGaugeJNI.getManufacturer(m_handle);
  }

  /**
   * Gets the manufacturer identifier of the battery.
   *
   * @return Battery manufacturer reported by the BFG
   */
  public BatteryManufacturer getManufacturer() {
    BatteryManufacturer state;
    int rawState = getRawManufacturer();

    if (rawState == 0) state = BatteryManufacturer.Duracell;
    else if (rawState == 1) state = BatteryManufacturer.Energizer;
    else if (rawState == 2) state = BatteryManufacturer.Interstate;
    else if (rawState == 3) state = BatteryManufacturer.MightyMax;
    else if (rawState == 4) state = BatteryManufacturer.MKPowered;
    else if (rawState == 5) state = BatteryManufacturer.PowerSonic;
    else state = BatteryManufacturer.NumMfgs;

    return state;
  }

  /**
   * Sets the user-defined nickname of this battery.
   *
   * @param newName New battery nickname
   */
  public void setNickname(String newName) {
    BattFuelGaugeJNI.setNickname(m_handle, newName);
  }

  /**
   * Sets the raw battery manufacturer
   * FIXED: Added this method
   *
   * @param newMfg New raw battery manufacturer
   */
  public void setRawManufacturer(int rawMfg) {
    BattFuelGaugeJNI.setManufacturer(m_handle, rawMfg);
  }

  /**
   * Sets the battery manufacturer
   *
   * @param newMfg New battery manufacturer
   */
  public void setManufacturer(BatteryManufacturer newMfg) {
    int rawMfg;

    if (newMfg == BatteryManufacturer.Duracell) rawMfg = 0;
    else if (newMfg == BatteryManufacturer.Energizer) rawMfg = 1;
    else if (newMfg == BatteryManufacturer.Interstate) rawMfg = 2;
    else if (newMfg == BatteryManufacturer.MightyMax) rawMfg = 3;
    else if (newMfg == BatteryManufacturer.MKPowered) rawMfg = 4;
    else if (newMfg == BatteryManufacturer.PowerSonic) rawMfg = 5;
    else rawMfg = 6;

    BattFuelGaugeJNI.setManufacturer(m_handle, rawMfg);
  }


  // -------------------------------------------------------------------------
  // Last full charge cycle statistics
  // -------------------------------------------------------------------------

  /**
   * Returns the RMS discharge current from the most recent full charge cycle.
   *
   * @return RMS discharge current in amperes (A)
   */
  public float getCycleRmsDischargeCurrent() {
    return BattFuelGaugeJNI.getCycleRmsDischargeCurrent(m_handle);
  }

  /**
   * Returns the highest charging current seen in the last charge cycle.
   *
   * @return maximum charging current in amperes (A)
   */
  public float getCycleMaxCurrent() {
    return BattFuelGaugeJNI.getCycleMaxCurrent(m_handle);
  }

  /**
   * Returns the most negative (highest discharge) current from the last charge cycle.
   *
   * @return minimum current in amperes (A)
   */
  public float getCycleMinCurrent() {
    return BattFuelGaugeJNI.getCycleMinCurrent(m_handle);
  }

  /**
   * Returns the lowest voltage recorded during the last charge cycle.
   *
   * @return minimum voltage in volts (V)
   */
  public float getCycleMinVoltage() {
    return BattFuelGaugeJNI.getCycleMinVoltage(m_handle);
  }

  /**
   * Returns the highest voltage recorded during the last charge cycle.
   *
   * @return maximum voltage in volts (V)
   */
  public float getCycleMaxVoltage() {
    return BattFuelGaugeJNI.getCycleMaxVoltage(m_handle);
  }

  /**
   * Returns the lowest depth of discharge reached in the last charge cycle.
   *
   * @return minimum depth of discharge in ampere-hours (Ah)
   */
  public float getCycleMinDodAh() {
    return BattFuelGaugeJNI.getCycleMinDodAh(m_handle);
  }

  /**
   * Returns the highest depth of discharge reached in the last charge cycle.
   *
   * @return maximum depth of discharge in ampere-hours (Ah)
   */
  public float getCycleMaxDodAh() {
    return BattFuelGaugeJNI.getCycleMaxDodAh(m_handle);
  }

  /**
   * Returns the lowest energy discharged in the last charge cycle.
   *
   * @return minimum discharged energy in watt-hours (Wh)
   */
  public float getCycleMinDodWh() {
    return BattFuelGaugeJNI.getCycleMinDodWh(m_handle);
  }

  /**
   * Returns the highest energy discharged in the last charge cycle.
   *
   * @return maximum discharged energy in watt-hours (Wh)
   */
  public float getCycleMaxDodWh() {
    return BattFuelGaugeJNI.getCycleMaxDodWh(m_handle);
  }

  /**
   * Returns how long the battery was charging during the last full cycle.
   *
   * @return charging time in seconds
   */
  public float getCycleChargeTime() {
    return BattFuelGaugeJNI.getCycleChargeTime(m_handle);
  }

  /**
   * Returns how long the battery was discharging during the last full cycle.
   *
   * @return discharging time in seconds
   */
  public float getCycleDischargeTime() {
    return BattFuelGaugeJNI.getCycleDischargeTime(m_handle);
  }

  /**
   * Returns the voltage at the start of the most recent charge cycle.
   *
   * @return starting voltage in volts (V)
   */
  public float getCycleStartingVoltage() {
    return BattFuelGaugeJNI.getCycleStartingVoltage(m_handle);
  }

  // -------------------------------------------------------------------------
  // Last robot match statistics
  // -------------------------------------------------------------------------

  /**
   * Returns the RMS discharge current observed during the last robot match.
   *
   * @return RMS discharge current in amperes (A)
   */
  public float getMatchRmsDischargeCurrent() {
    return BattFuelGaugeJNI.getMatchRmsDischargeCurrent(m_handle);
  }

  /**
   * Returns the peak current (highest magnitude) during the last match.
   *
   * @return maximum current in amperes (A)
   */
  public float getMatchMaxCurrent() {
    return BattFuelGaugeJNI.getMatchMaxCurrent(m_handle);
  }

  /**
   * Returns the most negative (highest discharge) current from the last match.
   *
   * @return minimum current in amperes (A)
   */
  public float getMatchMinCurrent() {
    return BattFuelGaugeJNI.getMatchMinCurrent(m_handle);
  }

  /**
   * Returns the lowest voltage seen during the last robot match.
   *
   * @return minimum voltage in volts (V)
   */
  public float getMatchMinVoltage() {
    return BattFuelGaugeJNI.getMatchMinVoltage(m_handle);
  }

  /**
   * Returns the highest voltage seen during the last robot match.
   *
   * @return maximum voltage in volts (V)
   */
  public float getMatchMaxVoltage() {
    return BattFuelGaugeJNI.getMatchMaxVoltage(m_handle);
  }

  /**
   * Returns the lowest depth of discharge reached during the last match.
   *
   * @return minimum depth of discharge in ampere-hours (Ah)
   */
  public float getMatchMinDodAh() {
    return BattFuelGaugeJNI.getMatchMinDodAh(m_handle);
  }

  /**
   * Returns the highest depth of discharge reached during the last match.
   *
   * @return maximum depth of discharge in ampere-hours (Ah)
   */
  public float getMatchMaxDodAh() {
    return BattFuelGaugeJNI.getMatchMaxDodAh(m_handle);
  }

  /**
   * Returns total charging time (if any) during the last robot match.
   *
   * @return charging time in seconds
   */
  public float getMatchChargeTime() {
    return BattFuelGaugeJNI.getMatchChargeTime(m_handle);
  }

  /**
   * Returns total discharging time during the last robot match.
   *
   * @return discharging time in seconds
   */
  public float getMatchDischargeTime() {
    return BattFuelGaugeJNI.getMatchDischargeTime(m_handle);
  }

  /**
   * Returns the battery voltage at the beginning of the last robot match.
   *
   * @return starting voltage in volts (V)
   */
  public float getMatchStartingVoltage() {
    return BattFuelGaugeJNI.getMatchStartingVoltage(m_handle);
  }

  /**
   * Returns the stored energy at the start of the last robot match.
   *
   * @return starting energy in watt-hours (Wh)
   */
  public float getMatchStartingEnergy() {
    return BattFuelGaugeJNI.getMatchStartingEnergy(m_handle);
  }

  /**
   * Returns the remaining energy at the end of the last robot match.
   *
   * @return ending energy in watt-hours (Wh)
   */
  public float getMatchEndingEnergy() {
    return BattFuelGaugeJNI.getMatchEndingEnergy(m_handle);
  }

  /**
   * Returns the stored charge at the beginning of the last robot match.
   *
   * @return starting charge in ampere-hours (Ah)
   */
  public float getMatchStartingCharge() {
    return BattFuelGaugeJNI.getMatchStartingCharge(m_handle);
  }

  /**
   * Returns the remaining charge at the end of the last robot match.
   *
   * @return ending charge in ampere-hours (Ah)
   */
  public float getMatchEndingCharge() {
    return BattFuelGaugeJNI.getMatchEndingCharge(m_handle);
  }

  /**
   * Returns the total duration of the last robot match.
   *
   * @return match duration in seconds
   */
  public float getMatchDuration() {
    return BattFuelGaugeJNI.getMatchDuration(m_handle);
  }
  
  /**
   * Determine if BFG is present on the CAN bus and at least one CAN message
   * was received the past two seconds
   * 
   * @return TRUE if at least on CAN message was received from sensor within the past two seconds, otherwise false
   */
  public boolean isConnected() {
    return BattFuelGaugeJNI.isConnected(m_handle);
  }
}
