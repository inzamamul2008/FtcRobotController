package org.firstinspires.ftc.teamcode;


public final class Constants {

    public static final class Drivetrain {
        public static final String LEFT_FRONT_DRIVE_MOTOR = "frontLeft";
        public static final String LEFT_BACK_DRIVE_MOTOR = "backLeft";
        public static final String RIGHT_FRONT_DRIVE_MOTOR = "frontRight";
        public static final String RIGHT_BACK_DRIVE_MOTOR = "backRight";
        public static final String IMU_NAME = "imu";
    }

    public static final class Shooter {
        // Hardware Map Names
        public static final String LEFT_SHOOTER_MOTOR = "lShooter";
        public static final String RIGHT_SHOOTER_MOTOR = "rShooter";

        // --- FINAL TUNED VALUES WILL GO HERE ---
        // SHOOTER PID GAINS
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;
        public static final double SHOOTER_F_GAIN = 0.0;

        // --- PHYSICAL PROPERTIES ---
        public static final double SHOOTER_MOTOR_MAX_RPM = 6000;
        public static final double SHOOTER_MOTOR_MAX_VELOCITY = 5600; // Ticks per sec
        public static final double SHOOTER_MOTOR_MAX_PPR = 28;
        public static final double SHOOTER_GEAR_RATIO = 2;
    }



    public static final class Index {
        // Hardware Map Names
        public static final String INDEX_MOTOR = "index"; // Renamed for clarity

        // INDEX
        public static final double INDEX_P_GAIN = 0.5;
        public static final double INDEX_I_GAIN = 0.0;
        public static final double INDEX_D_GAIN = 0.0;
        public static final double INDEX_F_GAIN = 0.0;

        // ---PHYSICAL PROPERTIES---
        public static final double INDEX_MOTOR_MAX_RPM = 1150;
        public static final double INDEX_MOTOR_MAX_VELOCITY = 805; // Ticks per sec
        public static final double INDEX_MOTOR_GEAR_RATIO = 1.5; // Change
        public static final double INDEX_MOTOR_MAX_PPR = 28;
    }

    public static final class AutoAlign {
        public static final double ALIGN_P_GAIN = 0;
        public static final double ALIGN_I_GAIN = 0;
        public static final double ALIGN_D_GAIN = 0;
    }
}