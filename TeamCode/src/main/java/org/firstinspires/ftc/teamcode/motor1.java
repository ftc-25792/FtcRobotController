package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="BasicJavaLessonTest", group="Autonomous")
public class Motor1 extends LinearOpMode {
    private DcMotor leftFrontMotor;


    private static final double FORWARD_DISTANCE = 0.5;
    private static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        driveForward(1);
        sleep(1000);
        driveForward(-1);
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);

        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");

    }

    private void driveForward(double distance) {
        setMotorPower(DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);
        sleep(1500);
    }
}

