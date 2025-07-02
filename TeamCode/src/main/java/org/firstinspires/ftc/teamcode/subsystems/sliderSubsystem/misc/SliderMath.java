package org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.misc;

// Essentially, this group of functions help me convert from ticks to linear velocity
// (centimeters) and vice versa, making it easier to keep track of how these values are managed
public class SliderMath {
    // Constructor and setup
    private final double ticksPerCm;

    /* The constructor takes three parameters:
     *
     * @ticksPerRevolution, which represents how many ticks are found within one motor rotation
     * @linearTravelPerRevolution, which represents how much distance (in cm) the elevator travels
     * for one complete motor rotation
     * @gearRatioReduction, which represents the reduction in the motors
     *
     */
    public SliderMath(int ticksPerRevolution, double linearTravelPerRevolution, double gearRatioReduction) {
        /* The formula is divided into two parts:
         *
         * First, ticksPerRevolution / linearTravelPerRevolution. This part calculates the amount
         * of ticks per centimeter of linear travel per motor revolution.
         *
         * Then, * gearRatioReduction; if it is a value like 5.2 (meaning 5.2 motor revolutions)
         * for 1 output revolution), then multiplying by it would mean you need more motor ticks
         * for the same linear travel, which makes sense.
         *
         * With the values set in SliderConstants.java, it should return ~12.13243
         *
         */
        this.ticksPerCm = (ticksPerRevolution / linearTravelPerRevolution) * gearRatioReduction;
    }

    // This function converts from distance in centimeters to ticks
    public int toTicks(double distanceInCm) {
        // An easy calculation. I simply multiply the amount of ticks there are in a cm times the
        // distance in cm too
        return (int) Math.round(distanceInCm * ticksPerCm);
    }

    // This function converts from ticks to centimeters
    public double toCm(double ticks) {
        return ticks / ticksPerCm;
    }
}

// Emilio Nájera — May-31st-2025