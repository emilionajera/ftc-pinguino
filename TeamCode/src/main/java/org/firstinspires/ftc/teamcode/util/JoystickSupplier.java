package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Supplier;

// Built this class in order to avoid excessive <Pair> and <Supplier> elements in my code lmao
// This also allows me to manage more effectively the way information is transmitted by passing only one object

// The following version is compatible with FTC since it is based on supported Java versions
// Simplified version using Java 17+ records:
// public record JoystickSupplier(Supplier<Double> x, Supplier<Double> y) {}
public class JoystickSupplier {
    private final Supplier<Double> x;
    private final Supplier<Double> y;

    public JoystickSupplier(Supplier<Double> x, Supplier<Double> y) {
        this.x = x;
        this.y = y;
    }

    public Supplier<Double> getX() {
        return x;
    }

    public Supplier<Double> getY() {
        return y;
    }
}

// Emilio Nájera, May—29th—2025