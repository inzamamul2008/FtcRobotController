package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp")
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        DcMotor indexMotor = hardwareMap.dcMotor.get("index");
        DcMotor lShooterMotor = hardwareMap.dcMotor.get("lShooter");
        DcMotor rShooterMotor = hardwareMap.dcMotor.get("rShooter");


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        indexMotor.setDirection(DcMotor.Direction.REVERSE);
        rShooterMotor.setDirection(DcMotor.Direction.REVERSE);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            setSafePower(frontLeftMotor,  frontLeftPower);
            setSafePower(backLeftMotor,   backLeftPower);
            setSafePower(frontRightMotor, frontRightPower);
            setSafePower(backRightMotor,  backRightPower);

            // Intake
            setSafePower(intakeMotor, gamepad1.right_trigger);
            setSafePower(lShooterMotor, gamepad1.left_trigger);
            setSafePower(rShooterMotor, gamepad1.left_trigger);


            // Shooter + Indexer
            if (gamepad1.a) {
                setSafePower(lShooterMotor, 1.0);
                setSafePower(rShooterMotor, 1.0);
                setSafePower(indexMotor, 1.0);
            } else {
                setSafePower(lShooterMotor, 0);
                setSafePower(rShooterMotor, 0);
                setSafePower(indexMotor, 0);
            }
        }
    }

    void setSafePower(DcMotor motor, double targetPower) {
        final double SLEW_RATE = 0.2;
        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));
        motor.setPower(currentPower += limitedChange);
    }
}