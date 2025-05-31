package org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc;


// This method allows us to convert easily from ticks to degrees and vice versa
public class JointMath {

    // Constructor and setup
    private final double ticksPerDegree;

    // Takes the number of ticks necessary to reach what would be the equivalent to 90º and then
    // divides it by 90 to get how many ticks are equal to one degree without considering gear ratio.
    // I then apply reduction by dividing this result by the gear ratio and then we get how many ticks are
    // within one degree.
    public JointMath(double ticksFor90, double gearRatioReduction) {
        this.ticksPerDegree = (ticksFor90 / 90) / gearRatioReduction;
    }

    // Convert from degrees to ticks
    public int toTicks(double degrees) {
        // An easy calculation. I simply multiply the number of degrees by how many ticks are there in a degree
        return (int) Math.round(degrees * ticksPerDegree);
    }

    // Convert from ticks to degrees
    public double toDegrees(int ticks) {
        return ticks / ticksPerDegree;
    }
}

// Emilio Nájera, May—30th—2025