package org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc;

public final class JointConstants {

    // Declaring Setup constants, like IDs
    public static final class Setup {
        public static final String rightJointMotorId = "jointRightMotor"; //todo check these ids
        public static final String leftJointMotorId = "jointLeftMotor";

        // This number returns how many ticks are necessary to reach 180 degrees in a motor encoder
        // Having this number allows me to make several calculations in order to reach a concrete angle (JointMath.java)
        public static final int ticksFor180 = 2800; // todo check this number
    }

    // Declaring limits to avoid breaking and cooking the motor
    public static final class MeasureLimits {
        // minTicksAllowed defines the minimum ticks the encoder can take at the lowest angle permitted.
        // This is be the equivalent to the joint being at 0 degrees
        public static final double minTicksAllowed = 17.1; //todo check these numbers

        // maxTicksAllowed defines the maximum ticks the encoder can take at the greatest angle permitted.
        // This is be the equivalent to the joint being at 90 degrees
        public static final double maxTicksAllowed = 2819238.9; // todo check these numbers
    }
}

// Emilio Nájera — May-30th-2025