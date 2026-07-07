package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * BoatHookExtensionTest
 * --------------------------------------------------------------------------------
 * Test / showcase OpMode for the spool-driven Carbon Fiber Boat Hook extension.
 *
 * Mechanism overview:
 *   - Instead of a linear slide, the hook is wound onto a spool and rolls out.
 *   - ms (motor spool) drives the spool. It is geared 48:100 (output spins slower
 *     than the motor, more torque) and is the stronger motor for RETRACTING the
 *     hook back onto the spool.
 *   - mw (motor wheel) drives a friction wheel that pulls the carbon fiber hook
 *     off the spool. It is geared 40:80 and is the stronger motor for EXTENDING
 *     the hook outward.
 *   - Both motors are GoBilda motors rated at 1150 RPM (free-run, pre-gearbox-stage
 *     RPM as configured on the motor itself before the external gear reduction
 *     below is applied).
 *   - Running both motors together is more efficient than running either alone,
 *     since each is "good" at one direction and "bad" at the other.
 *
 * Controls:
 *   - Right Bumper  -> RETRACT. Both ms and mw run together at FULL speed, as
 *                      NEGATIVE power (-1.0 on both). Bumper is digital
 *                      (on/off), so retract is NOT variable speed.
 *   - Right Trigger -> EXTEND. mw always runs at full trigger speed (matches
 *                      trigger depth 1:1, positive power), while ms runs at an
 *                      independently adjustable power level (see Dpad Up/Down
 *                      below), also scaled by trigger depth, positive power.
 *   - Dpad Up       -> While extending, increases the ms extend-power level by
 *                      +0.1 per press (edge-detected: one press = one step).
 *   - Dpad Down     -> While extending, decreases the ms extend-power level by
 *                      -0.1 per press (edge-detected: one press = one step).
 *   - Both motors are expected to stall naturally at full extension and full
 *     retraction, so no soft limits or encoder-based cutoffs are implemented.
 *
 * ms extend-power level:
 *   - Starts at 0.5 (i.e. ms runs at half power relative to full trigger when
 *     extending, while mw runs at full trigger power).
 *   - Adjustable in fixed 0.1 increments via Dpad Up/Down, clamped to [0.0, 1.0].
 *   - This value is independent of mw and does NOT reset when switching between
 *     extend and retract - it persists for the rest of the OpMode run until
 *     changed again.
 *   - This only affects EXTEND (right trigger). RETRACT (right bumper) always
 *     drives both motors equally at full power.
 *
 * Telemetry:
 *   - Raw encoder ticks for ms and mw
 *   - Motor-shaft revolutions for ms and mw (ticks / ticksPerMotorRev)
 *   - Output-shaft (spool / wheel) revolutions after external gearing
 *   - Applied power for each motor
 *   - Current ms extend-power level
 *
 * Use this OpMode to run full extend / full retract cycles and record the
 * revolution counts shown in telemetry, and to tune the ms extend-power level
 * live to find the best balance between mw and ms while extending.
 * --------------------------------------------------------------------------------
 */
@TeleOp(name = "Boat Hook Extension Test", group = "Test")
public class BoatHookExtensionTest extends LinearOpMode {

    // ---------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------
    private DcMotorEx ms; // motor spool   (geared 48:100, stronger retracting)
    private DcMotorEx mw; // motor wheel   (geared 40:80,  stronger extending)

    // ---------------------------------------------------------------------------
    // Gearing / encoder constants
    // ---------------------------------------------------------------------------
    // GoBilda motors: ticks-per-revolution depends on the specific gearbox cartridge
    // on the motor itself. 1150 RPM corresponds to GoBilda's 5203/5202 series 1:1
    // (no internal gearbox reduction beyond what's encoded here) at 28 ticks per
    // revolution of the encoder shaft (pre-internal-gearbox). If your specific
    // motor's internal gearbox differs, update TICKS_PER_MOTOR_REV to match the
    // value from the GoBilda spec sheet for your exact part number.
    private static final double TICKS_PER_MOTOR_REV = 28.0;

    // External gear reduction stage (driver teeth : driven teeth), applied AFTER
    // the motor's own internal gearbox/encoder. Spool gearing: 48 (driver) -> 100 (driven)
    private static final double SPOOL_GEAR_DRIVER = 48.0;
    private static final double SPOOL_GEAR_DRIVEN = 100.0;
    private static final double SPOOL_EXTERNAL_RATIO = SPOOL_GEAR_DRIVER / SPOOL_GEAR_DRIVEN; // <1.0 = torque increase

    // Wheel gearing: 40 (driver) -> 80 (driven)
    private static final double WHEEL_GEAR_DRIVER = 40.0;
    private static final double WHEEL_GEAR_DRIVEN = 80.0;
    private static final double WHEEL_EXTERNAL_RATIO = WHEEL_GEAR_DRIVER / WHEEL_GEAR_DRIVEN; // <1.0 = torque increase

    // Ticks per OUTPUT-shaft revolution (spool revolution / wheel revolution),
    // accounting for the external gear stage.
    private static final double TICKS_PER_SPOOL_REV = TICKS_PER_MOTOR_REV / SPOOL_EXTERNAL_RATIO;
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV / WHEEL_EXTERNAL_RATIO;

    // ---------------------------------------------------------------------------
    // ms extend-power level (right trigger / EXTEND only)
    // ---------------------------------------------------------------------------
    // Starts at 0.5 power, adjustable in +/-0.1 steps via Dpad Up/Down, clamped
    // to [0.0, 1.0]. Persists across mode switches (does not reset on retract).
    private double msExtendPower = 0.5;
    private static final double MS_EXTEND_POWER_STEP = 0.1;
    private static final double MS_EXTEND_POWER_MIN = 0.0;
    private static final double MS_EXTEND_POWER_MAX = 1.0;

    // ---------------------------------------------------------------------------
    // mw retract-power level (right bumper / RETRACT only)
    // ---------------------------------------------------------------------------
    // Starts at 1.0 (full power), adjustable in +/-0.1 steps via Dpad Left/Right,
    // clamped to [0.0, 1.0]. Persists across mode switches. ms always stays at
    // fixed full power (-1.0) during retract regardless of this value.
    private double mwRetractPower = 1.0;
    private static final double MW_RETRACT_POWER_STEP = 0.1;
    private static final double MW_RETRACT_POWER_MIN = 0.0;
    private static final double MW_RETRACT_POWER_MAX = 1.0;

    // Edge-detection helpers for dpad buttons (so a single press = single step)
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------------
        // Hardware mapping
        // -----------------------------------------------------------------------
        ms = hardwareMap.get(DcMotorEx.class, "ms");
        mw = hardwareMap.get(DcMotorEx.class, "mw");

        // Positive power = EXTEND, negative power = RETRACT on this build.
        // If a motor still spins backwards from what you expect, flip just
        // that one line to REVERSE rather than rewriting the trigger logic.
        ms.setDirection(DcMotor.Direction.REVERSE);
        mw.setDirection(DcMotor.Direction.REVERSE);

        // Brake when at zero power so the hook doesn't drift open/closed under its
        // own weight or spring tension between inputs.
        ms.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        telemetry.addLine("Boat Hook Extension Test - Ready");
        telemetry.addLine("Right Bumper = RETRACT (ms fixed full power, mw at adjustable power)");
        telemetry.addLine("Right Trigger = EXTEND (mw full, ms at adjustable power)");
        telemetry.addLine("Dpad Up/Down = adjust ms extend power +/-0.1");
        telemetry.addLine("Dpad Left/Right = adjust mw retract power +/-0.1");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------------------------------------------------------
            // Dpad Up / Down: adjust ms extend-power level, edge-detected
            // so one press = one 0.1 step. Persists across mode switches.
            // ---------------------------------------------------------------
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev) {
                msExtendPower += MS_EXTEND_POWER_STEP;
                if (msExtendPower > MS_EXTEND_POWER_MAX) {
                    msExtendPower = MS_EXTEND_POWER_MAX;
                }
            }
            dpadUpPrev = dpadUpNow;

            boolean dpadDownNow = gamepad1.dpad_down;
            if (dpadDownNow && !dpadDownPrev) {
                msExtendPower -= MS_EXTEND_POWER_STEP;
                if (msExtendPower < MS_EXTEND_POWER_MIN) {
                    msExtendPower = MS_EXTEND_POWER_MIN;
                }
            }
            dpadDownPrev = dpadDownNow;

            // ---------------------------------------------------------------
            // Dpad Left / Right: adjust mw retract-power level, edge-detected
            // so one press = one 0.1 step. Persists across mode switches.
            // ms always stays at fixed full power (-1.0) during retract.
            // ---------------------------------------------------------------
            boolean dpadRightNow = gamepad1.dpad_right;
            if (dpadRightNow && !dpadRightPrev) {
                mwRetractPower += MW_RETRACT_POWER_STEP;
                if (mwRetractPower > MW_RETRACT_POWER_MAX) {
                    mwRetractPower = MW_RETRACT_POWER_MAX;
                }
            }
            dpadRightPrev = dpadRightNow;

            boolean dpadLeftNow = gamepad1.dpad_left;
            if (dpadLeftNow && !dpadLeftPrev) {
                mwRetractPower -= MW_RETRACT_POWER_STEP;
                if (mwRetractPower < MW_RETRACT_POWER_MIN) {
                    mwRetractPower = MW_RETRACT_POWER_MIN;
                }
            }
            dpadLeftPrev = dpadLeftNow;

            // ---------------------------------------------------------------
            // Read inputs
            // ---------------------------------------------------------------
            boolean retractButton = gamepad1.right_bumper; // RETRACT: ms + mw equal, full power (digital)
            double extendTrigger = gamepad1.right_trigger;  // EXTEND: mw full, ms scaled

            double msPower;
            double mwPower;

            if (retractButton) {
                // RETRACT: ms always runs at fixed full power (-1.0). mw runs
                // at the independently adjustable mwRetractPower level
                // (negative, since retract is negative power).
                msPower = -1.0;
                mwPower = -mwRetractPower;
            } else {
                // EXTEND: mw matches trigger depth 1:1, ms scaled by the
                // independently adjustable msExtendPower level.
                mwPower = extendTrigger;
                msPower = extendTrigger * msExtendPower;
            }

            mw.setPower(mwPower);
            ms.setPower(msPower);

            // ---------------------------------------------------------------
            // Telemetry: raw ticks + computed revolutions for data logging
            // ---------------------------------------------------------------
            int msTicks = ms.getCurrentPosition();
            int mwTicks = mw.getCurrentPosition();

            double msMotorRevs = msTicks / TICKS_PER_MOTOR_REV;
            double mwMotorRevs = mwTicks / TICKS_PER_MOTOR_REV;

            double spoolOutputRevs = msTicks / TICKS_PER_SPOOL_REV;
            double wheelOutputRevs = mwTicks / TICKS_PER_WHEEL_REV;

            telemetry.addData("ms extend power level", "%.1f", msExtendPower);
            telemetry.addData("mw retract power level", "%.1f", mwRetractPower);
            telemetry.addLine("--- ms (spool motor) ---");
            telemetry.addData("ms power", "%.2f", msPower);
            telemetry.addData("ms ticks", msTicks);
            telemetry.addData("ms motor revs", "%.3f", msMotorRevs);
            telemetry.addData("ms spool (output) revs", "%.3f", spoolOutputRevs);
            telemetry.addLine("--- mw (wheel motor) ---");
            telemetry.addData("mw power", "%.2f", mwPower);
            telemetry.addData("mw ticks", mwTicks);
            telemetry.addData("mw motor revs", "%.3f", mwMotorRevs);
            telemetry.addData("mw wheel (output) revs", "%.3f", wheelOutputRevs);
            telemetry.addLine();
            telemetry.addLine("R Bumper = Retract | R Trigger = Extend");
            telemetry.addLine("Dpad Up/Down = ms extend power +/-0.1 | Dpad Left/Right = mw retract power +/-0.1");
            telemetry.update();
        }
    }

    /**
     * Stops both motors, resets encoders to zero, then re-enables encoder-based
     * (RUN_USING_ENCODER) power control so power commands still work normally
     * after the reset.
     */
    private void resetEncoders() {
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}