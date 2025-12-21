package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class ApriltagWebcam1 extends OpMode {
    AprilTagWebcam apriltagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        apriltagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop () {
        apriltagWebcam.update();
        AprilTagDetection id20 = apriltagWebcam.getTagBySpecificId(20);
        apriltagWebcam.displayDetectionTelemetry(id20);

    }

}
