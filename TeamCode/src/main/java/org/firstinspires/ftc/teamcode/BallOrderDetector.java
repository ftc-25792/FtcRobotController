package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

class BallOrderDetector {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public BallOrderDetector(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public String getBallOrder() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return "UNKNOWN";
        int id = detections.get(0).id;
        if (id == 22) return "PGP";
        if (id == 23) return "PPG";
        if (id == 21) return "GPP";
        return "UNKNOWN";
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}


