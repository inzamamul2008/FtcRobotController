package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Auto Shoot")
public class AutoShooter extends LinearOpMode {

    // Vision
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Shooter motors
    DcMotorEx shooterMotor;
    DcMotorEx feederMotor;

    @Override
    public void runOpMode() {

        // Initialize motors
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        feederMotor  = hardwareMap.get(DcMotorEx.class, "feeder");

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        feederMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize camera
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("Ready - waiting for start");
        telemetry.update();

        waitForStart();

        boolean hasShot = false;

        while (opModeIsActive() && !hasShot) {

            aprilTagWebcam.update();

            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(20);

            if (tag != null) {

                // --- AIM CHECK ---
                boolean aimed = Math.abs(tag.ftcPose.bearing) < 2.0;

                // --- DISTANCE-BASED VELOCITY ---
                double distance = tag.ftcPose.range;
                double targetVelocity = 20 * distance + 1200; // TUNE THIS

                telemetry.addData("Tag Seen", tag.id);
                telemetry.addData("Distance (cm)", distance);
                telemetry.addData("Target Velocity", targetVelocity);

                if (aimed) {

                    // Spin up shooter
                    shooterMotor.setVelocity(targetVelocity);

                    double currentVelocity = shooterMotor.getVelocity();
                    telemetry.addData("Shooter Velocity", currentVelocity);

                    // Feed ONLY when shooter is at speed
                    if (currentVelocity >= targetVelocity * 0.95) {

                        feederMotor.setPower(0.7);
                        sleep(400); // feed ONE ball

                        feederMotor.setPower(0);
                        shooterMotor.setVelocity(0);

                        hasShot = true; // prevents multiple shots
                    }
                }
            }

            telemetry.update();
        }

        aprilTagWebcam.stop();
    }
}
