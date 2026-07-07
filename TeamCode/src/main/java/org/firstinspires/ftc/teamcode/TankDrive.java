package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tank Drive TeleOp
 *
 * Hardware:
 *   - "left_drive"  : GoBILDA motor driving the entire left side
 *   - "right_drive" : GoBILDA motor driving the entire right side
 *
 * Controls:
 *   - Left  joystick Y  → forward / backward
 *   - Right joystick X  → rotate (turn in place)
 *
 * RPM Balancing:
 *   Both encoders are read every loop. A simple proportional correction
 *   trims whichever side is running faster so both sides match when
 *   driving straight.  The correction is suppressed during sharp turns
 *   where sides are intentionally running at different speeds.
 */
@TeleOp(name = "Tank Drive", group = "Drive")
public class TankDrive extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // ── RPM-balancing constants ───────────────────────────────────────────────

    /** GoBILDA 5203 series: 537.7 PPR at the output shaft (26.9:1 gear ratio).
     *  Change this to match whichever GoBILDA motor variant you are using:
     *    312 RPM motor → 537.7 PPR
     *    223 RPM motor → 751.8 PPR
     *    435 RPM motor → 383.6 PPR
     */
    private static final double COUNTS_PER_REV = 537.7;

    /** How strongly to correct RPM mismatch (0 = off, 1 = maximum).
     *  Start around 0.04–0.08 and tune on the field. */
    private static final double RPM_KP = 0.05;

    /** Ignore RPM error below this threshold (counts/sec) to avoid
     *  jitter at very low speeds. */
    private static final double RPM_DEADBAND = 20.0; // counts per second

    /** Only apply correction when both sides are commanded above this
     *  power level — avoids fighting intentional turns. */
    private static final double CORRECTION_MIN_POWER = 0.15;

    /** Maximum power adjustment the RPM correction can apply. */
    private static final double MAX_CORRECTION = 0.10;

    // ── Joystick dead zone ────────────────────────────────────────────────────
    private static final double JOYSTICK_DEADZONE = 0.05;

    // ── Timing ────────────────────────────────────────────────────────────────
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ── Encoder state ────────────────────────────────────────────────────────
    private int lastLeftPos  = 0;
    private int lastRightPos = 0;

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {

        // ── Init hardware ─────────────────────────────────────────────────────
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // One side will be physically reversed depending on how the motors
        // are mounted. Flip whichever side drives backward when commanded forward.
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when power is zero — keeps the robot from coasting
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders, then switch to RUN_WITHOUT_ENCODER so we can
        // apply our own RPM correction rather than the built-in PID.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        loopTimer.reset();
        lastLeftPos  = leftDrive.getCurrentPosition();
        lastRightPos = rightDrive.getCurrentPosition();

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) dt = 0.020; // safety guard against divide-by-zero

            // ── Read joysticks ────────────────────────────────────────────────
            double drive  = -applyDeadzone(gamepad1.left_stick_y);  // push up = positive
            double rotate = -applyDeadzone(gamepad1.right_stick_x);
            // ── Basic tank mix ────────────────────────────────────────────────
            double leftPower  = drive + rotate;
            double rightPower = drive - rotate;

            // Normalize so neither side exceeds ±1
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower  /= maxPower;
                rightPower /= maxPower;
            }

            // ── RPM correction (straight-line only) ───────────────────────────
            double correction = 0.0;

            boolean drivingStraight = Math.abs(drive) < 0.05   // spinning in place, not driving forward
                    && Math.abs(rotate) > CORRECTION_MIN_POWER  // actually rotating
                    || Math.abs(rotate) < 0.05                  // OR driving straight
                    && Math.abs(leftPower)  > CORRECTION_MIN_POWER
                    && Math.abs(rightPower) > CORRECTION_MIN_POWER;
            if (drivingStraight) {
                int currentLeftPos  = leftDrive.getCurrentPosition();
                int currentRightPos = rightDrive.getCurrentPosition();

                double leftVelocity  = (currentLeftPos  - lastLeftPos)  / dt; // counts/sec
                double rightVelocity = (currentRightPos - lastRightPos) / dt; // counts/sec

                lastLeftPos  = currentLeftPos;
                lastRightPos = currentRightPos;

// During a point turn, sides spin opposite directions — compare magnitudes
                boolean pointTurn = Math.abs(drive) < 0.05 && Math.abs(rotate) > CORRECTION_MIN_POWER;
                double velocityError = pointTurn
                        ? Math.abs(leftVelocity) - Math.abs(rightVelocity)
                        : leftVelocity - rightVelocity;

                if (Math.abs(velocityError) > RPM_DEADBAND) {
                    correction = RPM_KP * velocityError;
                    correction = Math.max(-MAX_CORRECTION, Math.min(MAX_CORRECTION, correction));
                }

                // Slow down whichever side is running faster
                leftPower  -= correction;
                rightPower += correction;
            } else {
                // Still update encoder snapshot so the next straight-line
                // measurement starts fresh
                lastLeftPos  = leftDrive.getCurrentPosition();
                lastRightPos = rightDrive.getCurrentPosition();
            }

            // ── Apply power ───────────────────────────────────────────────────
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // ── Telemetry ─────────────────────────────────────────────────────
            double leftRPM  = countsPerSecToRPM(
                    (leftDrive.getCurrentPosition()  - lastLeftPos)  / dt);
            double rightRPM = countsPerSecToRPM(
                    (rightDrive.getCurrentPosition() - lastRightPos) / dt);

            telemetry.addData("Drive / Rotate", "%.2f  /  %.2f", drive, rotate);
            telemetry.addData("Power  L / R",   "%.3f  /  %.3f", leftPower, rightPower);
            telemetry.addData("RPM    L / R",   "%.1f  /  %.1f", leftRPM, rightRPM);
            telemetry.addData("RPM correction", "%.4f", correction);
            telemetry.addData("Enc    L / R",
                    "%d  /  %d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors cleanly
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Returns 0 if the input is within the deadzone, otherwise passes it through. */
    private double applyDeadzone(double value) {
        return Math.abs(value) < JOYSTICK_DEADZONE ? 0.0 : value;
    }

    /** Converts encoder counts/second to shaft RPM. */
    private double countsPerSecToRPM(double countsPerSec) {
        return (countsPerSec / COUNTS_PER_REV) * 60.0;
    }
}