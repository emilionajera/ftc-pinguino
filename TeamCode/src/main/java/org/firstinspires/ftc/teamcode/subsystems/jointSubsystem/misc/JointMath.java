package org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc;


// This method allows us to convert easily from ticks to degrees and vice versa
public class JointMath {

    // Constructor and setup
    private final int ticksPerDegree;

    public JointMath(int ticksFor180) {
        this.ticksPerDegree = ticksFor180;
    }

    // Convert from degrees to ticks
    public int toTicks(double degrees) {
        return (int) Math.round(degrees * ticksPerDegree);
    }

    // Convert from ticks to degrees
    public double toDegrees(int ticks) {
        return (double) ticks / ticksPerDegree;
    }
}

// Emilio Nájera, May—30th—2025