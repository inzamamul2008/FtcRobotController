package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Auto Shoot", group = "Autonomous")
public class AutoShooter extends LinearOpMode {

    // Vision
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Shooter motors
    DcMotorEx lShooter;
    DcMotorEx rShooter;
    DcMotorEx index;

    @Override
    public void runOpMode() {

        // --- HARDWARE INIT ---
        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");
        index    = hardwareMap.get(DcMotorEx.class, "index");

        // Shooter setup
        lShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // One shooter motor usually needs reversing
        lShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rShooter.setDirection(DcMotorEx.Direction.REVERSE);

        index.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Vision init
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("AprilTag Auto Shoot Ready");
        telemetry.update();

        waitForStart();

        boolean hasShot = false;

        while (opModeIsActive() && !hasShot) {

            aprilTagWebcam.update();
            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(20);

            if (tag != null) {

                double distance = tag.ftcPose.range;
                double bearing  = tag.ftcPose.bearing;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (cm)", distance);
                telemetry.addData("Bearing (deg)", bearing);

                // --- AIM CHECK ---
                boolean aimed = Math.abs(bearing) < 2.0;

                // --- DISTANCE → VELOCITY (TUNE THIS) ---
                double targetVelocity = (20 * distance) + 1200;

                if (aimed) {

                    // Spin shooter
                    lShooter.setVelocity(targetVelocity);
                    rShooter.setVelocity(targetVelocity);

                    // Use slower motor for safety
                    double currentVelocity = Math.min(
                            lShooter.getVelocity(),
                            rShooter.getVelocity()
                    );

                    telemetry.addData("Target Velocity", targetVelocity);
                    telemetry.addData("Current Velocity", currentVelocity);

                    // Feed ONLY when shooter is at speed
                    if (currentVelocity >= targetVelocity * 0.95) {

                        index.setPower(0.7);
                        sleep(400);   // feeds ONE ball

                        index.setPower(0);
                        lShooter.setVelocity(0);
                        rShooter.setVelocity(0);

                        hasShot = true; // prevents double shooting
                    }
                }
            }

            telemetry.update();
        }

        aprilTagWebcam.stop();
    }
}
