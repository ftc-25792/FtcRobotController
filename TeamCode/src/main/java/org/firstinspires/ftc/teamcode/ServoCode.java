package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoCode", group="TeleOp")
public class ServoCode extends LinearOpMode {

    // Declare a Servo object for the intake servo
    Servo intakeServo;

    @Override
    public void runOpMode() {
        // Initialize the hardware map
        intakeServo = hardwareMap.get(Servo.class, "intake_servo"); //Rename this servo to our servo name

        waitForStart();

        while (opModeIsActive()) {
            // Control the servo position based on gamepad input
            if (gamepad1.a) {
                // Open the intake servo
                intakeServo.setPosition(0.0);
            } else if (gamepad1.b) {
                // Close the intake servo
                intakeServo.setPosition(1.0);
            }
        }
    }
}