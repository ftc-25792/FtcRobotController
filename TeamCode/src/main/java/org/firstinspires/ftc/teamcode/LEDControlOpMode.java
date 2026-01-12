package org.firstinspires.ftc.teamcode; // Replace with your team's package name

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name="Blinkin Control OpMode", group="TeleOp")
public class LEDControlOpMode extends LinearOpMode {

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); // Name must match config

        // Set an initial pattern
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // The loop where you change patterns dynamically
        while (opModeIsActive()) {
            // Example: Change pattern based on gamepad input
            if (gamepad1.a) {
                setNewPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (gamepad1.b) {
                setNewPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }

            telemetry.addData("Current Pattern", pattern.toString());
            telemetry.update();
        }
    }

    // Helper method to set the pattern and update telemetry
    private void setNewPattern(RevBlinkinLedDriver.BlinkinPattern newPattern) {
        pattern = newPattern;
        blinkinLedDriver.setPattern(pattern);
    }
}
