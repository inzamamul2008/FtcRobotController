package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    IMU imu;

    public double integral = 0;
    public double lastError = 0;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        index = hardwareMap.get(DcMotorEx.class, "index");
        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");

        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        lShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));


    }



    public double shooterPIDCalculate(double shooterTargetVel, double getShooterVel, long lastTime) {
        // Calculate error
        double error = shooterTargetVel - getShooterVel();
        // Calculate time difference
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Proportional term
        double proportional = Constants.Shooter.SHOOTER_P_GAIN * error;

        // Integral term
        integral += error * dt;
        double integralTerm = Constants.Shooter.SHOOTER_I_GAIN * integral;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double derivativeTerm = Constants.Shooter.SHOOTER_D_GAIN * derivative;
        lastError = error;

        // Calculate output
        double output = proportional + integralTerm + derivativeTerm;

        // Clamp output to valid range [-1, 1]

        return output;
    }

    // --- Setters ---
    /*public void setRPMtoVel(double rpm) {
        double rpm_gRatio = rpm * Shooter.SHOOTER_GEAR_RATIO;
        double rps = rpm_gRatio / 60; // Rotations per sec
        double vel = rps * Shooter.SHOOTER_MOTOR_MAX_PPR; // Ticks per sec
        setShooterVel(vel);
    }*/
    public void setShooterVel(double vel) {
        lShooter.setVelocity(vel);
        rShooter.setVelocity(vel);
    }
    public void setShooter(double speed) {
        lShooter.setPower(speed);
        rShooter.setPower(speed);
    }
    public void shooterOff() {
        lShooter.setPower(0);
        rShooter.setPower(0);
    }

    // --- Getters ---
    public double getShooterVel() {
        return (lShooter.getVelocity() + rShooter.getVelocity()) / 2;
    }
    public double getShooterSpeed() {
        return (lShooter.getPower() + rShooter.getPower()) / 2;
    }

    // --- Setters ---
    public void setIntakeVel(double vel) {
        intake.setVelocity(vel);
    }
    public void setIntake(double speed) {
        intake.setPower(speed);
    }
    public void intakeOff() {
        intake.setPower(0);
    }
    public void intakeReverse() {
        intake.setPower(-0.7);
    }

    public void setIndexVel(double vel) {
        index.setVelocity(vel);
    }
    public void setIndex(double speed) {
        index.setPower(speed);
    }
    public void indexOff() {
        index.setPower(0);
    }

    public void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    public void indexBallUp() {
        setIndexPos(getIndexPos() + 150);
    }
    public void indexBallDown() {
        setIndexPos(getIndexPos() - 150);
    }

    // --- Getters ---
    public double getIntakeVel() {
        return intake.getVelocity();
    }
    public double getIntakeSpeed() {
        return intake.getPower();
    }

    public double getIndexVel() {
        return index.getVelocity();
    }

    public double getIndexSpeed() {
        return index.getPower();
    }
    public int getIndexPos() {
        return index.getCurrentPosition();
    }
}