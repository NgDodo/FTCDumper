package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * SwerveChassis — Four-module swerve with full (FRC-style) kinematics.
 *
 * Hardware:
 *   "m0"/"s0", "m1"/"s1", "m2"/"s2", "m3"/"s3" — drive motor / steer servo per module.
 *   Steer angle sources:
 *     m0, m1 -> AS5600 over I2C ("enc0", "enc1", on SEPARATE buses), absolute, 4096 CPR
 *     m2, m3 -> REV Through Bore quadrature on the drive motor's encoder port, 8192 CPR
 *
 * Controls:
 *   Left  stick   — Translation (FIELD-centric): stick direction = travel
 *                   direction on the field, regardless of robot heading.
 *   Right stick X — Rotation rate (clockwise positive). Combinable with
 *                   translation — each module gets its own vector-summed
 *                   angle and speed, like FRC swerve.
 *   Right Bumper  — Re-zero all pods.
 *   Options (☰)   — Reset field heading (point robot "away from driver" first).
 *
 * Behavior:
 *   - Pods hold their last commanded heading when idle.
 *   - Wheel-flip optimization: a pod never steers more than 90°; if the target
 *     is further, it flips 180° and reverses drive instead (standard swerve).
 *   - Steer-settle delay: when a pod's heading error is large, its drive power
 *     is suppressed for up to DRIVE_DELAY_MS so the pod can turn before the
 *     wheel drives. Drive resumes early once the pod is within
 *     STEER_SETTLE_TOLERANCE_DEG of target.
 */
@TeleOp(name = "Swerve Chassis", group = "Drive")
public class SwerveChassisTest extends LinearOpMode {

    private static final int AS5600_COUNTS_PER_REV = 4096;   // m0, m1
    private static final int TBE_COUNTS_PER_REV     = 8192;  // m2, m3 (REV Through Bore, quadrature)

    // AS5600 modules (m0, m1) — 4096 CPR, so ~2x the gain of the TBE pods for equal stiffness.
    private static final double AS5600_kP = 0.0004;
    private static final double AS5600_kD = 0.0000;

    // TBE modules (m2, m3) — 8192 CPR.
    private static final double TBE_kP    = 0.0002;
    private static final double TBE_kD    = 0.0000;

    // Steer power sign per encoder type. +1 if encoder counts UP when the servo
    // drives the pod in its positive direction; -1 if they disagree (runaway/wrap).
    private static final double AS5600_STEER_SIGN = +1.0;
    private static final double TBE_STEER_SIGN    = +1.0;

    // Hard-coded AS5600 raw zero offsets (0..4095), measured with the wheel
    // physically pointing forward. These survive restarts, unlike zeroHere().
    private static final int ENC0_ZERO_OFFSET = 0;   // TODO: measure m0 forward
    private static final int ENC1_ZERO_OFFSET = 0;   // TODO: measure m1 forward

    private static final double MAX_STEER_POWER = 1;
    private static final double DEADBAND        = 0.05;

    // ---- steer-settle drive delay ----
    // While a pod's heading error exceeds the tolerance, its drive power is held
    // at 0 for up to DRIVE_DELAY_MS (then drives anyway so the robot can't stall
    // forever on a pod that never quite settles).
    private static final long   DRIVE_DELAY_MS             = 0;
    private static final double STEER_SETTLE_TOLERANCE_DEG = 20.0;

    // ---- wheel-flip optimization ----
    // If the target heading is more than 90° away, steer to target+180° and
    // reverse drive instead. Max steer travel becomes 90°. Set false to disable.
    private static final boolean OPTIMIZE_FLIP = true;

    // ---- field-centric drive ----
    // Uses the Control Hub's built-in IMU (BHI260AP). Set false for robot-centric.
    private static final boolean FIELD_CENTRIC = true;

    // Per-side drive direction. Applied as a multiplier on drive power only —
    // deliberately NOT motor.setDirection(REVERSE), because reversing m3's
    // motor would also flip its Through Bore encoder counts and break steering.
    private static final double LEFT_DRIVE_SIGN  = +1.0;   // m0, m2
    private static final double RIGHT_DRIVE_SIGN = -1.0;   // m1, m3 (reversed)

    // How the Control Hub is mounted on the robot — MUST match reality or the
    // heading will be wrong. Adjust these two to your hub's actual orientation.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private IMU imu;
    private SwerveModule[] modules;
    private long lastTime = 0;

    @Override
    public void runOpMode() {

        // Module positions as unit vectors from robot center (+x right, +y forward).
        // These set each pod's rotation direction contribution; a square chassis
        // just needs the normalized corner directions.
        final double C = Math.sqrt(0.5);   // 1/sqrt(2)

        modules = new SwerveModule[] {
                new SwerveModule("FL m0", hardwareMap.get(DcMotorEx.class, "m0"),
                        hardwareMap.get(CRServo.class, "s0"),
                        new AbsoluteAngleSource(hardwareMap.get(PandaEncoder.class, "enc0"),
                                AS5600_COUNTS_PER_REV, AS5600_kP, AS5600_kD,
                                AS5600_STEER_SIGN, ENC0_ZERO_OFFSET),
                        -C, +C, LEFT_DRIVE_SIGN),   // front-left
                new SwerveModule("FR m1", hardwareMap.get(DcMotorEx.class, "m1"),
                        hardwareMap.get(CRServo.class, "s1"),
                        new AbsoluteAngleSource(hardwareMap.get(PandaEncoder.class, "enc1"),
                                AS5600_COUNTS_PER_REV, AS5600_kP, AS5600_kD,
                                AS5600_STEER_SIGN, ENC1_ZERO_OFFSET),
                        +C, +C, RIGHT_DRIVE_SIGN),   // front-right
                new SwerveModule("BL m2", hardwareMap.get(DcMotorEx.class, "m2"),
                        hardwareMap.get(CRServo.class, "s2"),
                        new QuadratureAngleSource(hardwareMap.get(DcMotorEx.class, "m2"),
                                TBE_COUNTS_PER_REV, TBE_kP, TBE_kD, TBE_STEER_SIGN),
                        -C, -C, LEFT_DRIVE_SIGN),   // back-left
                new SwerveModule("BR m3", hardwareMap.get(DcMotorEx.class, "m3"),
                        hardwareMap.get(CRServo.class, "s3"),
                        new QuadratureAngleSource(hardwareMap.get(DcMotorEx.class, "m3"),
                                TBE_COUNTS_PER_REV, TBE_kP, TBE_kD, TBE_STEER_SIGN),
                        +C, -C, RIGHT_DRIVE_SIGN),   // back-right
        };

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(HUB_LOGO_DIR, HUB_USB_DIR)));
        imu.resetYaw();

        telemetry.addLine("Ready. Right Bumper re-zeros pods; Options resets heading.");
        telemetry.update();

        waitForStart();

        // Seed each pod's held target to where it currently sits, so nothing
        // snaps to 0° before the first stick input.
        for (SwerveModule m : modules) m.seedHoldToCurrent();

        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;
            if (dt <= 0 || dt > 0.1) dt = 0.02;

            if (gamepad1.right_bumper) {
                for (SwerveModule m : modules) m.rezero();
            }
            if (gamepad1.options) {
                imu.resetYaw();   // point robot away from driver, press to re-home field frame
            }

            double vx     =  gamepad1.left_stick_x;    // right positive
            double vy     = -gamepad1.left_stick_y;    // forward positive
            double rotate =  gamepad1.right_stick_x;   // clockwise positive

            if (Math.hypot(vx, vy)   < DEADBAND) { vx = 0; vy = 0; }
            if (Math.abs(rotate)     < DEADBAND) { rotate = 0; }

            // ---- field-centric: rotate the translation vector from the field
            // frame into the robot frame using the IMU yaw (CCW positive). ----
            double yawDeg = 0;
            if (FIELD_CENTRIC) {
                yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double yawRad = Math.toRadians(yawDeg);
                double cos = Math.cos(yawRad), sin = Math.sin(yawRad);
                double rvx =  vx * cos + vy * sin;
                double rvy = -vx * sin + vy * cos;
                vx = rvx;
                vy = rvy;
            }

            boolean hasCmd = (vx != 0 || vy != 0 || rotate != 0);

            // ---- swerve inverse kinematics ----
            // Each module's velocity = translation + rotation contribution.
            // Rotation (CW positive) at unit position (px, py) contributes
            // (py, -px) * rotate.
            double[] speeds = new double[modules.length];
            double[] angles = new double[modules.length];
            double maxSpeed = 0;

            for (int i = 0; i < modules.length; i++) {
                double mvx = vx + rotate * modules[i].posY;
                double mvy = vy - rotate * modules[i].posX;
                speeds[i] = Math.hypot(mvx, mvy);
                angles[i] = (Math.toDegrees(Math.atan2(mvx, mvy)) + 360.0) % 360.0;
                maxSpeed = Math.max(maxSpeed, speeds[i]);
            }

            // Keep ratios intact if any module exceeds full power.
            if (maxSpeed > 1.0) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= maxSpeed;
            }

            for (int i = 0; i < modules.length; i++) {
                modules[i].update(hasCmd, angles[i], speeds[i], dt, now);
                modules[i].addTelemetry(telemetry);
            }
            telemetry.addData("Yaw (°)", "%.1f", yawDeg);
            telemetry.update();
        }

        for (SwerveModule m : modules) m.stop();
    }

    // ===== angle sources (all report raw counts 0..CPR-1) =====

    private interface AngleSource {
        int currentCounts();
        int countsPerRev();
        double kP();
        double kD();
        double steerSign();
        void rezero();
    }

    /** AS5600 absolute magnetic encoder over I2C. Returns raw counts referenced to its zero offset. */
    private static class AbsoluteAngleSource implements AngleSource {
        final PandaEncoder enc;
        final int countsPerRev;
        final double kP, kD, steerSign;
        AbsoluteAngleSource(PandaEncoder enc, int countsPerRev, double kP, double kD,
                            double steerSign, int rawZeroOffset) {
            this.enc = enc;
            this.countsPerRev = countsPerRev;
            this.kP = kP;
            this.kD = kD;
            this.steerSign = steerSign;
            // Apply the hard-coded offset so heading survives restarts.
            enc.setZeroOffset(rawZeroOffset);
        }
        @Override public int currentCounts() {
            return (int) Math.round(enc.getHeadingDegrees() / 360.0 * countsPerRev) % countsPerRev;
        }
        @Override public int countsPerRev() { return countsPerRev; }
        @Override public double kP() { return kP; }
        @Override public double kD() { return kD; }
        @Override public double steerSign() { return steerSign; }
        @Override public void rezero() { enc.zeroHere(); }
    }

    /** Quadrature encoder on a drive motor's encoder port. */
    private static class QuadratureAngleSource implements AngleSource {
        final DcMotorEx motor;
        final int countsPerRev;
        final double kP, kD, steerSign;
        QuadratureAngleSource(DcMotorEx motor, int countsPerRev, double kP, double kD,
                              double steerSign) {
            this.motor = motor;
            this.countsPerRev = countsPerRev;
            this.kP = kP;
            this.kD = kD;
            this.steerSign = steerSign;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        @Override public int currentCounts() {
            return ((motor.getCurrentPosition() % countsPerRev) + countsPerRev) % countsPerRev;
        }
        @Override public int countsPerRev() { return countsPerRev; }
        @Override public double kP() { return kP; }
        @Override public double kD() { return kD; }
        @Override public double steerSign() { return steerSign; }
        @Override public void rezero() {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ===== module =====

    private static class SwerveModule {

        final String      label;
        final DcMotorEx   driveMotor;
        final CRServo     steerServo;
        final AngleSource angle;
        final double      posX, posY;   // unit position from robot center
        final double      driveSign;    // +1 or -1, flips drive power only

        private double prevError = 0;
        private double lastCommandedDeg = 0;   // heading the pod holds when idle

        // steer-settle drive delay state
        private boolean settling      = false;
        private long    settleStartNs = 0;

        private double telTargetDeg = 0, telCurrentDeg = 0, telDrivePower = 0;
        private int    telError = 0;
        private double telSteerPower = 0;
        private boolean telHeld = false;

        SwerveModule(String label, DcMotorEx driveMotor, CRServo steerServo,
                     AngleSource angle, double posX, double posY, double driveSign) {
            this.label      = label;
            this.driveMotor = driveMotor;
            this.steerServo = steerServo;
            this.angle      = angle;
            this.posX       = posX;
            this.posY       = posY;
            this.driveSign  = driveSign;

            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /** Set the held heading to wherever the pod physically is right now. */
        void seedHoldToCurrent() {
            lastCommandedDeg = angle.currentCounts() / (double) angle.countsPerRev() * 360.0;
        }

        void rezero() {
            angle.rezero();
            prevError = 0;
            settling  = false;
            seedHoldToCurrent();
        }

        /**
         * @param hasCmd     true if any stick input is active
         * @param cmdAngleDeg  this module's kinematic target heading (deg, 0..360)
         * @param cmdSpeed     this module's kinematic drive speed (0..1)
         */
        void update(boolean hasCmd, double cmdAngleDeg, double cmdSpeed,
                    double dt, long nowNs) {

            int    cpr         = angle.countsPerRev();
            int    currentNorm = angle.currentCounts();
            double currentDeg  = currentNorm / (double) cpr * 360.0;

            double drivePower;
            if (hasCmd && cmdSpeed > 1e-6) {
                lastCommandedDeg = cmdAngleDeg;
                drivePower       = cmdSpeed;
            } else if (hasCmd) {
                // Pure-rotation edge case where this module's vector is ~zero
                // (only possible off-center); hold heading, no drive.
                drivePower = 0;
            } else {
                drivePower = 0;   // idle: hold last heading
            }

            double targetDeg    = lastCommandedDeg;
            int    targetCounts = (int) Math.round(targetDeg / 360.0 * cpr) % cpr;
            int    error        = shortestError(currentNorm, targetCounts, cpr);

            // Wheel-flip optimization: never steer more than 90°; flip the
            // target 180° and reverse drive instead.
            if (OPTIMIZE_FLIP && Math.abs(error) > cpr / 4) {
                targetCounts = (targetCounts + cpr / 2) % cpr;
                error        = shortestError(currentNorm, targetCounts, cpr);
                drivePower   = -drivePower;
                targetDeg    = targetCounts / (double) cpr * 360.0;
            }

            // Steer-settle delay: suppress drive while heading error is large,
            // for at most DRIVE_DELAY_MS per settle event.
            int settleTolCounts = (int) (STEER_SETTLE_TOLERANCE_DEG / 360.0 * cpr);
            boolean held = false;
            if (Math.abs(error) > settleTolCounts && drivePower != 0) {
                if (!settling) {
                    settling      = true;
                    settleStartNs = nowNs;
                }
                if ((nowNs - settleStartNs) / 1_000_000L < DRIVE_DELAY_MS) {
                    drivePower = 0;
                    held = true;
                }
            } else {
                settling = false;
            }

            double derivative = (error - prevError) / dt;
            double steerPower = Range.clip(
                    angle.steerSign() * (angle.kP() * error + angle.kD() * derivative),
                    -MAX_STEER_POWER, MAX_STEER_POWER);
            prevError = error;

            steerServo.setPower(steerPower);
            driveMotor.setPower(driveSign * drivePower);

            telTargetDeg  = targetDeg;
            telCurrentDeg = currentDeg;
            telDrivePower = drivePower;
            telError      = error;
            telSteerPower = steerPower;
            telHeld       = held;
        }

        void addTelemetry(Telemetry telemetry) {
            telemetry.addLine("--- " + label + (telHeld ? "  [SETTLING]" : "") + " ---");
            telemetry.addData("Target  (°)", "%.1f", telTargetDeg);
            telemetry.addData("Current (°)", "%.1f", telCurrentDeg);
            telemetry.addData("Drive      ", "%.2f", telDrivePower);
            telemetry.addData("Err (cts)  ", telError);
            telemetry.addData("SteerPwr   ", "%.4f", telSteerPower);
        }

        void stop() {
            driveMotor.setPower(0);
            steerServo.setPower(0);
        }

        private static int shortestError(int current, int target, int cpr) {
            int error = target - current;
            if (error >  cpr / 2) error -= cpr;
            if (error < -cpr / 2) error += cpr;
            return error;
        }
    }
}