package org.firstinspires.ftc.teamcode.util;

public class MathUtils {

    // Declaring clamping functions for double & int datatypes
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}

// Emilio Nájera, May—30th—2025