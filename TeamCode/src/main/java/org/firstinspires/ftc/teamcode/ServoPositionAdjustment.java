package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositionAdjust", group = "Testing")
public class ServoPositionAdjustment extends LinearOpMode {

    private Servo flapperLeft, flapperRight;

    // Set YOUR REAL starting values here.
    // These will be used at INIT and will NOT move unless you change them.
    private double flapperLeftPosition = 0.5;
    private double flapperRightPosition = 0.5;

    @Override
    public void runOpMode() {
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");

        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        // **Set servos ONCE at INIT — no getPosition() (that caused the snap)**
        flapperLeft.setPosition(flapperLeftPosition);
        flapperRight.setPosition(flapperRightPosition);

        // Show the STARTING positions
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Left Start", flapperLeftPosition);
        telemetry.addData("Right Start", flapperRightPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Increase/Decrease left
            if (gamepad1.a) flapperLeftPosition += 0.01;
            if (gamepad1.b) flapperLeftPosition -= 0.01;

            // Increase/Decrease right
            if (gamepad1.x) flapperRightPosition += 0.01;
            if (gamepad1.y) flapperRightPosition -= 0.01;

            // Clamp values
            flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
            flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));

            // Apply updated positions
            flapperLeft.setPosition(flapperLeftPosition);
            flapperRight.setPosition(flapperRightPosition);

            // Telemetry
            telemetry.addData("Flapper Left Pos", "%.3f", flapperLeftPosition);
            telemetry.addData("Flapper Right Pos", "%.3f", flapperRightPosition);
            telemetry.update();
        }
    }
}
