package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Panda Encoder Test", group = "test")
public class PandaEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        PandaEncoder encoder = hardwareMap.get(PandaEncoder.class, "steerEncoder");

        // Bring-up check before START
        telemetry.addLine("AS5600 bring-up");
        telemetry.addData("Magnet detected", encoder.magnetDetected());
        telemetry.addData("Too weak (move closer)", encoder.magnetTooWeak());
        telemetry.addData("Too strong (move away)", encoder.magnetTooStrong());
        telemetry.addLine();
        telemetry.addLine("Press START, then turn the shaft by hand.");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            timer.reset();
            encoder.update(); // keeps the multi-turn total current
            double msPerRead = timer.milliseconds();
            telemetry.addData("avg read (ms)", "%.3f", msPerRead);

            telemetry.addData("Magnet OK", encoder.magnetOk());
            telemetry.addData("Raw (0-4095)", encoder.readRawAngle());
            telemetry.addData("Heading (0-360)", "%.1f", encoder.getHeadingDegrees());
            telemetry.addData("Signed (-180..180)", "%.1f", encoder.getHeadingDegreesSigned());
            telemetry.addData("Total turns", "%.2f", encoder.getTotalRevolutions());

            if (gamepad1.a) encoder.zeroHere(); // hold A to set current position as zero
            telemetry.addLine("(hold A to zero at current position)");

            telemetry.update();
        }
    }
}