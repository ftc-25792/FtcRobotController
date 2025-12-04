package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "DistanceSensorTest", group = "Sensor")
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor distanceSensor;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor leftRear;


    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
       distanceSensor = hardwareMap.get(DistanceSensor.class, "check_distance");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 30, set the power to 0.3
            if (distanceSensor.getDistance(DistanceUnit.CM) < 30) {
                leftFront.setPower(0.3);
                rightFront.setPower(0.3);
                leftRear.setPower(0.3);
                rightRear.setPower(0.3);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }
        }
    }
}