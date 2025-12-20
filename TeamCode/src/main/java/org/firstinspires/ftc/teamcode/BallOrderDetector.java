package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class BallOrderDetector {

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

    /**
     * Returns ball order based on AprilTag ID
     * 22 → PGP
     * 23 → PPG
     * 21 → GPP
     */
    public String getBallOrder() {

        if (aprilTag == null) return "UNKNOWN";

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            return "UNKNOWN";
        }

        // Take the FIRST visible tag (auto-safe)
        int id = detections.get(0).id;

        switch (id) {
            case 22: return "PGP";
            case 23: return "PPG";
            case 21: return "GPP";
            default: return "UNKNOWN";
        }
    }

    /** Shut down camera cleanly */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
