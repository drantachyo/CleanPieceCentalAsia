package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

public class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    public Vision(HardwareMap hw) {
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }

    public AprilTagDetection getTarget(int id) {
        if (aprilTag.getDetections() == null) return null;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.id == id && d.metadata != null) return d;
        }
        return null;
    }
    public void stop() {
        if (portal != null) {
            portal.close(); // Теперь переменная "используется"
        }
    }
}