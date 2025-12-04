package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Color Sensor HW")
public class ColorSensorHw extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {

            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                telemetry.addData("Color", "Red Detected");
            }
            else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                telemetry.addData("Color", "Blue Detected");
            }
            else if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) {
                telemetry.addData("Color", "Green Detected");
            }
            else {
                telemetry.addData("Color", "Unknown");
            }

            telemetry.update();
        }
    }
}
