package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Motor Encoder Alignment Test", group="Test")
public class MotorEncoderAlignmentTest extends LinearOpMode {

    // List your drivetrain motors here
    private DcMotor frontLeft, frontRight, backLeft, backRight, launcherLeft, launcherRight;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");

        // Stop and reset all encoders
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, launcherLeft, launcherRight};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setPower(0);
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addLine("Press Play to test motors one by one...");
        telemetry.update();
        waitForStart();

        // Test each motor individually
        for (DcMotor m : motors) {
            if (!opModeIsActive()) break;

            telemetry.addLine("Testing motor: " + getMotorName(m));
            telemetry.addLine("Get Direction" + m.getDirection());
            telemetry.addLine("Wheel should spin forward, observe sign of encoder");
            telemetry.update();

            sleep(1000);

            // Reset encoder
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Apply small power
            m.setPower(0.2);
            sleep(1000);
            m.setPower(0);

            int encoderPos = m.getCurrentPosition();
            telemetry.addData("Encoder value", encoderPos);

            if (encoderPos > 0) {
                telemetry.addLine("✅ Encoder counts increase. Motor direction OK.");
            } else if (encoderPos < 0) {
                telemetry.addLine("⚠ Encoder counts decrease. You may need to REVERSE this motor.");
            } else {
                telemetry.addLine("❌ Encoder did not change! Check wiring or motor.");
            }

            telemetry.update();

            // Pause before next motor
            sleep(5000);
        }

        telemetry.addLine("Test complete. Use above results to set DcMotor.Direction.");
        telemetry.update();

        while (opModeIsActive()) {
            idle(); // keep opmode alive to see telemetry
        }
    }

    private String getMotorName(DcMotor motor) {
        // Simple helper to identify motor by variable
        if (motor == frontLeft) return "Front Left";
        if (motor == frontRight) return "Front Right";
        if (motor == backLeft) return "Back Left";
        if (motor == backRight) return "Back Right";
        if (motor == launcherLeft) return "Left Launcher";
        if (motor == launcherRight) return "Right Launcher";
        return "Unknown Motor";
    }
}
