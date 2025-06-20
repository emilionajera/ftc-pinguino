package org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc;

public final class JointConstants {

    // Declaring Setup constants, like IDs
    public static final class Setup {
        public static final String rightJointMotorId = "jointRightMotor";
        public static final String leftJointMotorId = "jointLeftMotor";
    }

    public static final class MotorProperties {
        // This number returns how many ticks are necessary to reach 180 degrees in a motor encoder
        // Having this number allows me to make several calculations in order to reach a concrete angle (JointMath.java)
        public static final int ticksFor90 = 280; // todo check this number

        // Got this number from the onShape CAD
        public static final double gearRatioReduction = 1.0;//(1.0 / 5) * (1.0 / 3) * (1.0 / 3);
    }

    // Declaring limits to avoid breaking and cooking the motor
    public static final class MeasureLimits {
        // minTicksAllowed defines the minimum ticks the encoder can take at the lowest angle permitted.
        // This is be the equivalent to the joint being at 0 degrees
        public static final int minTicksAllowed = 17; //todo check these numbers

        // maxTicksAllowed defines the maximum ticks the encoder can take at the greatest angle permitted.
        // This is be the equivalent to the joint being at 90 degrees
        public static final int maxTicksAllowed = 2819238; // todo check these numbers
    }
}

// Emilio Nájera — May-30th-2025