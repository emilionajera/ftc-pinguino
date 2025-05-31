package org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.misc;

public final class SliderConstants {

    // Declaring Setup constants, like IDs
    public static final class Setup {
        public static final String rightSliderMotorId = "sliderRightMotor"; //todo check these ids
        public static final String leftSliderMotorId = "sliderLeftMotor";
    }

    public static final class MotorProperties {
        // This first number returns how many ticks are there in a motor revolution.
        // The second number represents how much distance the elevator travels per revolution of the motor
        // Both of these numbers allow me to convert from ticks to linear velocity (centimeters)
        public static final int ticksPerRevolution = 1400; // todo check this number

        // This value is gotten from the circumference of the pulley attached to the motor
        // For example, if we had a pulley with a diameter of 5cm, then the circumference would be 5 * π,
        // which is about 15.7. This means that, for each revolution of the pulley, the belt moves 15.7 cm
        public static final double linearTravelPerRevolution = Math.PI * 12.5; // todo check this number. i got it from onShape CAD
        public static final double gearRatio = (1.0 / 5.2); // todo check that this is the correct reduction and not 3.7
    }

    // Declaring limits to avoid breaking and cooking the motor
    public static final class MeasureLimits {
        // minTicksAllowed defines the minimum ticks the encoder can take at the lowest distance permitted.
        // This is be the equivalent to the slider having travelled 0 cm (start position)
        public static final int minTicksAllowed = 17; //todo check these numbers

        // maxTicksAllowed defines the maximum ticks the encoder can take at the greatest distance permitted.
        // This is be the equivalent to the slider having travelled 20938457098 cm (max distance without breaking)
        public static final int maxTicksAllowed = 2819238; // todo check these numbers
    }
}

// Emilio Nájera — May-30th-2025