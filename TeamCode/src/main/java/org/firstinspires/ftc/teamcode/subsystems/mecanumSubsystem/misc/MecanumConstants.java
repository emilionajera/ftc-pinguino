package org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.misc;

public final class MecanumConstants {

    // Declaring Setup constants, like device IDs
    public static final class Setup {
        // Motors
        public static final String frontLeftMotorId = "mecanumFrontLeftMotor"; //todo check these ids
        public static final String frontRightMotorId = "mecanumFrontRightMotor";
        public static final String backLeftMotorId = "mecanumBackLeftMotor";
        public static final String backRightMotorId = "mecanumBackRightMotor";

        // Sensors
        public static final String imuId = "imu";
        public static final String otosId = "otos"; // same name will be used for PedroPathing
    }

    public static final class Values {
        // Assuming it is the bare motor with simply two stacked 3:1 reductions, it would be
        // RPM = 6000, Encoder Ticks per rev (Output Shaft) = 28 ->
        // Ticks/Second = (RPM/60) * Ticks per rev
        // Ticks/Second = 2800
        public static final int desiredTicksPerSecond = 2800;
    }
}

// Emilio Nájera — May-30th-2025