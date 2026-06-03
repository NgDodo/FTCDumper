package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Swerve Pod Test", group = "Test")
public class SwervePodTest extends LinearOpMode {

    // Controls the Gobilda drive motor — no encoder cable on this port.
    private DcMotorEx driveMotor;

    // CR servo for steering.
    private CRServo   steerServo;

    // Reads the REV Through Bore Encoder — mirrors how Sorter uses "bR".
    // In Driver Hub config: add a DC Motor named "steer_encoder" on whichever
    // hub port the TBE is plugged into. No motor needs to be on that port's
    // motor output — only the TBE on the encoder pins.
    private DcMotorEx steerEncoder;

    private static final int    COUNTS_PER_REV  = 12288;
    private static final int    ROTATION_TARGET = (int)(45.0 / 360.0 * COUNTS_PER_REV);

    private static final double kP              = 0.0002;
    private static final double kD              = 0.0000;
    private static final double MAX_STEER_POWER = 1;
    private static final double DEADBAND        = 0.05;

    private double prevError = 0;
    private long   lastTime  = 0;

    @Override
    public void runOpMode() {

        driveMotor   = hardwareMap.get(DcMotorEx.class, "drive_motor");
        steerServo   = hardwareMap.get(CRServo.class,   "steer_servo");
        steerEncoder = hardwareMap.get(DcMotorEx.class, "steer_encoder");

        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Identical to Sorter's sorterEncoder init
        steerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Right Bumper to re-zero.");
        telemetry.update();

        waitForStart();
        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            long   now = System.nanoTime();
            double dt  = (now - lastTime) / 1e9;
            lastTime   = now;
            if (dt <= 0 || dt > 0.1) dt = 0.02;

            if (gamepad1.right_bumper) {
                steerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                steerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                prevError = 0;
            }

            double stickX   =  gamepad1.left_stick_x;
            double stickY   = -gamepad1.left_stick_y;
            double stickMag =  Math.hypot(stickX, stickY);
            double rotate   =  gamepad1.right_stick_x;

            boolean hasTrans = stickMag        > DEADBAND;
            boolean hasRot   = Math.abs(rotate) > DEADBAND;

            // Read TBE — identical to how Sorter reads sorterEncoder
            int currentPos  = steerEncoder.getCurrentPosition();
            int currentNorm = normalize(currentPos);

            int    targetCounts;
            double drivePower;

            if (hasTrans) {
                double stickDeg = (Math.toDegrees(Math.atan2(stickX, stickY)) + 360.0) % 360.0;
                targetCounts = (int)(stickDeg / 360.0 * COUNTS_PER_REV);
                drivePower   = stickMag;

            } else if (hasRot) {
                targetCounts = ROTATION_TARGET;
                drivePower   = rotate;

            } else {
                targetCounts = currentNorm;
                drivePower   = 0;
            }

            int error = shortestError(currentNorm, targetCounts);

            double derivative = (error - prevError) / dt;
            double steerPower = Range.clip(kP * error + kD * derivative,
                    -MAX_STEER_POWER, MAX_STEER_POWER);
            prevError = error;

            steerServo.setPower(steerPower);
            driveMotor.setPower(drivePower);

            telemetry.addData("Target  (°)", "%.1f", (double) targetCounts / COUNTS_PER_REV * 360.0);
            telemetry.addData("Current (°)", "%.1f", (double) currentNorm   / COUNTS_PER_REV * 360.0);
            telemetry.addData("Raw pos     ", currentPos);
            telemetry.update();
        }

        driveMotor.setPower(0);
        steerServo.setPower(0);
    }

    // Identical to Sorter._normalize()
    private int normalize(int ticks) {
        return ((ticks % COUNTS_PER_REV) + COUNTS_PER_REV) % COUNTS_PER_REV;
    }

    // Identical to Sorter._calculateShortestError()
    private int shortestError(int current, int target) {
        int error = target - current;
        if (error >  COUNTS_PER_REV / 2) error -= COUNTS_PER_REV;
        if (error < -COUNTS_PER_REV / 2) error += COUNTS_PER_REV;
        return error;
    }
}