package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Supplier;

// Built this record in order to avoid excessive <Pair> and <Supplier> elements in my code lmao
// This also allows me to manage more effectively the way information is transmitted by passing only one object
public record JoystickSupplier(Supplier<Double> x, Supplier<Double> y) {}

// Emilio Nájera, May—29th—2025