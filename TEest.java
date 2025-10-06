package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST", group = "TeleOp")
public class TEest extends LinearOpMode {
    private DcMotor leftFrontMotor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");

        leftFrontMotor.setZeroPowerBehavior(BRAKE);

        if (gamepad1.right_bumper) {
            leftFrontMotor.setPower(1.0)
        }

        if (gamepad1.right_bumper) {
            leftFrontMotor.setPower(-1.0)
        }

        if (gamepad1.dpad_down) {
            leftFrontMotor.setPower(0.0)
        }

    }
}
