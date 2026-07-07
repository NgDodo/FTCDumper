package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * SingleTbePodTest — isolate ONE TBE module with the exact original two-pod logic.
 *
 * Purpose: determine whether the TBE misbehavior is code (the chassis
 * abstraction) or hardware/config. This op-mode deliberately uses NO
 * AngleSource, NO sign flip, NO hold-last-command — it is a near-verbatim copy
 * of the original working two-pod steering loop, driving a single module.
 *
 * Hardware:
 *   "m2" — drive motor with the REV Through Bore on its encoder port (quadrature)
 *   "s2" — steer CR servo
 *
 * Controls:
 *   Left stick   — Translation. Stick angle = wheel direction, magnitude = drive power.
 *   Right Bumper — Re-zero the encoder (sets current physical position as 0°).
 *
 * If this single pod steers cleanly and holds, the hardware/config is fine and
 * the problem is in the chassis abstraction. If it still fights/wraps/runs away,
 * the problem is hardware or config (encoder direction, port mapping, etc.).
 *
 * To test the other TBE pod, change "m2"/"s2" to "m3"/"s3".
 */
@TeleOp(name = "Single TBE Pod Test", group = "Test")
public class SingleTbePodTest extends LinearOpMode {

    // REV Through Bore in quadrature on the drive motor's encoder port.
    private static final int COUNTS_PER_REV = 8192;

    // Same gains as the original working two-pod code (raw-counts scale).
    private static final double kP              = 0.0002;
    private static final double kD              = 0.0000;
    private static final double MAX_STEER_POWER = 1;
    private static final double DEADBAND        = 0.05;

    private DcMotorEx driveMotor;
    private CRServo   steerServo;

    private double prevError = 0;
    private long   lastTime  = 0;

    @Override
    public void runOpMode() {

        driveMotor = hardwareMap.get(DcMotorEx.class, "m2");
        steerServo = hardwareMap.get(CRServo.class,   "s2");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Single TBE pod (m2/s2). Right Bumper to re-zero.");
        telemetry.addLine("Push left stick to steer; release to hold here.");
        telemetry.update();

        waitForStart();
        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;
            if (dt <= 0 || dt > 0.1) dt = 0.02;

            if (gamepad1.right_bumper) {
                driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                prevError = 0;
            }

            double stickX   =  gamepad1.left_stick_x;
            double stickY   = -gamepad1.left_stick_y;
            double stickMag =  Math.hypot(stickX, stickY);
            boolean hasTrans = stickMag > DEADBAND;

            // ----- identical math to the original two-pod loop -----
            int    currentNorm = normalize(driveMotor.getCurrentPosition());
            double currentDeg  = currentNorm / (double) COUNTS_PER_REV * 360.0;

            double targetDeg;
            double drivePower;
            if (hasTrans) {
                targetDeg  = (Math.toDegrees(Math.atan2(stickX, stickY)) + 360.0) % 360.0;
                drivePower = stickMag;
            } else {
                targetDeg  = currentDeg;   // original behavior: hold current when idle
                drivePower = 0;
            }

            int targetCounts = (int) (targetDeg / 360.0 * COUNTS_PER_REV);
            int error        = shortestError(currentNorm, targetCounts);

            double derivative = (error - prevError) / dt;
            double steerPower = Range.clip(kP * error + kD * derivative,
                    -MAX_STEER_POWER, MAX_STEER_POWER);
            prevError = error;

            steerServo.setPower(steerPower);
            driveMotor.setPower(drivePower);
            // -------------------------------------------------------

            telemetry.addData("Target  (°)", "%.1f", targetDeg);
            telemetry.addData("Current (°)", "%.1f", currentDeg);
            telemetry.addData("Raw counts ", driveMotor.getCurrentPosition());
            telemetry.addData("Err (cts)  ", error);
            telemetry.addData("SteerPwr   ", "%.4f", steerPower);
            telemetry.addData("Drive      ", "%.2f", drivePower);
            telemetry.update();
        }

        driveMotor.setPower(0);
        steerServo.setPower(0);
    }

    private static int normalize(int ticks) {
        return ((ticks % COUNTS_PER_REV) + COUNTS_PER_REV) % COUNTS_PER_REV;
    }

    private static int shortestError(int current, int target) {
        int error = target - current;
        if (error >  COUNTS_PER_REV / 2) error -= COUNTS_PER_REV;
        if (error < -COUNTS_PER_REV / 2) error += COUNTS_PER_REV;
        return error;
    }
}