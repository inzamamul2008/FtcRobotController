package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop_Basebot extends LinearOpMode {
    double direction_x, direction_y, pivot, heading;
    Project1Hardware robot;
    Gamepad gamepad;
    Gamepad lastGamepad;
    public boolean closeZone = true, farZone;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Project1Hardware();
        robot.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(robot);


        gamepad = new Gamepad();
        lastGamepad = new Gamepad();

        long lastTime = System.currentTimeMillis();

        waitForStart();
        robot.imu.resetYaw();

        while (opModeIsActive()) {

            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);


            direction_x = gamepad.left_stick_x;
            direction_y = gamepad.left_stick_y;
            pivot       = gamepad.right_stick_x * 0.8;
            heading     = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            if (gamepad.touchpad && !lastGamepad.touchpad) {
                robot.imu.resetYaw();
            }


            double shootCurVel = robot.getShooterVel();

            if (gamepad.triangle && !lastGamepad.triangle) {
                closeZone = true; farZone = false;
            } else if (gamepad.cross && !lastGamepad.cross) {
                farZone = true; closeZone = false;
            }

            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (closeZone) {
                    robot.setShooterVel(1000);
                } else if (farZone) {
                    robot.setShooterVel(1200);
                }
            } else if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                robot.shooterOff();
            }

            // --- Index Control ---
            if (gamepad.square && !lastGamepad.square) {
                robot.setIndexPos(robot.getIndexPos() + 280);
            } else if (gamepad.circle && !lastGamepad.circle) {
                robot.setIndexPos(robot.getIndexPos() - 280);
            }

            telemetry.addData("shooterVel", robot.getShooterVel());
            telemetry.addData("closeZone", closeZone);
            telemetry.addData("farZone", farZone);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
}