package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "AS5600 Encoder", xmlTag = "AS5600")
public class PandaEncoder extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // ---- AS5600 datasheet constants ----
    private static final I2cAddr ADDRESS  = I2cAddr.create7bit(0x36);
    private static final int REG_STATUS    = 0x0B;  // magnet status bits
    private static final int REG_RAW_ANGLE = 0x0C;  // 0x0C high, 0x0D low (12-bit, unscaled)
    private static final int COUNTS_PER_REV = 4096; // 12-bit

    // ---- state for the relative / multi-turn layer ----
    private int  lastRaw = -1;
    private long totalCounts = 0;
    private int  zeroOffset  = 0;   // raw count that should read as "0" (e.g. wheel forward)

    public PandaEncoder(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    // ===== raw read =====

    /** Absolute position within one turn, 0..4095, straight from the chip. */
    public int readRawAngle() {
        byte[] data = deviceClient.read(REG_RAW_ANGLE, 2); // [high, low]
        int high = data[0] & 0xFF;
        int low  = data[1] & 0xFF;
        return ((high << 8) | low) & 0x0FFF;   // mask to 12 bits
    }

    // ===== ABSOLUTE mode (use this for swerve steering) =====

    /** Absolute heading 0..360, referenced to the stored zero offset. */
    public double getHeadingDegrees() {
        int adjusted = ((readRawAngle() - zeroOffset) % COUNTS_PER_REV + COUNTS_PER_REV) % COUNTS_PER_REV;
        return adjusted * 360.0 / COUNTS_PER_REV;
    }

    /** Same heading as -180..+180, which steering controllers usually want. */
    public double getHeadingDegreesSigned() {
        double h = getHeadingDegrees();
        return (h > 180.0) ? h - 360.0 : h;
    }

    /** Record the current physical position as "0" (e.g. wheel pointing forward). */
    public void zeroHere() { zeroOffset = readRawAngle(); }

    /** Or set an offset you measured once and hard-coded (per module). */
    public void setZeroOffset(int rawOffset) { zeroOffset = rawOffset; }
    public int  getZeroOffset() { return zeroOffset; }

    // ===== RELATIVE mode (multi-turn total, e.g. for a drive wheel) =====

    /** Call once per loop to track rotation across full turns. */
    public void update() {
        int raw = readRawAngle();
        if (lastRaw < 0) lastRaw = raw;
        int delta = raw - lastRaw;
        if (delta >  COUNTS_PER_REV / 2) delta -= COUNTS_PER_REV;
        if (delta < -COUNTS_PER_REV / 2) delta += COUNTS_PER_REV;
        totalCounts += delta;
        lastRaw = raw;
    }

    public double getTotalRevolutions() { return totalCounts / (double) COUNTS_PER_REV; }
    public double getTotalDegrees()     { return totalCounts * 360.0 / COUNTS_PER_REV; }
    public void   resetTotal()          { totalCounts = 0; lastRaw = -1; }

    // ===== magnet health (AS5600 STATUS register) =====

    public int     readStatus()      { return deviceClient.read8(REG_STATUS) & 0xFF; }
    public boolean magnetDetected()  { return (readStatus() & 0x20) != 0; } // MD
    public boolean magnetTooWeak()   { return (readStatus() & 0x10) != 0; } // ML
    public boolean magnetTooStrong() { return (readStatus() & 0x08) != 0; } // MH
    public boolean magnetOk()        { int s = readStatus(); return (s & 0x20) != 0 && (s & 0x18) == 0; }

    // ===== required SDK overrides =====

    @Override protected boolean doInitialize() { return true; }
    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return "AS5600 Magnetic Encoder"; }
}