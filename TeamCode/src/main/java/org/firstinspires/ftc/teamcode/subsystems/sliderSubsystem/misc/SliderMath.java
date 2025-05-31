package org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.misc;

// Essentially, this group of functions help me convert from ticks to linear velocity
// (centimeters) and vice versa, making it easier to keep track of how these values are managed
public class SliderMath {
    // Constructor and setup
    private final double ticksPerCm;

    // The constructor takes three parameters:
    // ticksPerRevolution, which represents how many ticks are found within one motor rotation
    // linearTravelPerRevolution, which represents how much distance (in cm) the elevator travels for one complete motor rotation
    // gearRatioReduction, which represents the reduction in the motors
    public SliderMath(int ticksPerRevolution, double linearTravelPerRevolution, double gearRatioReduction) {
        this.ticksPerCm = ticksPerRevolution / (linearTravelPerRevolution * gearRatioReduction);
    }

    // This function converts from distance in centimeters to ticks
    public int toTicks(double distanceInCm) {
        // An easy calculation. I simply multiply the amount of ticks there are in a cm times the distance in cm too
        return (int) Math.round(distanceInCm * ticksPerCm);
    }

    // This function converts from centimeters to ticks
    public double toCm(double ticks) {
        return ticks / ticksPerCm;
    }
}

// Emilio Nájera — May-31st-2025