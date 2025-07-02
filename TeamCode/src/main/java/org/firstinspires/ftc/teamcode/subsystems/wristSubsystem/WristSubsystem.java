package org.firstinspires.ftc.teamcode.subsystems.wristSubsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem extends SubsystemBase {
    // Declaring useful variables
    Telemetry telemetry;

    // Declaring servomotors to be used
    ServoEx wristServo;


    // Constructor //
    public WristSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Setting up motors
        /*wristServo = new SimpleServo(
                hardwareMap,
        );*/
    }

    // Functional code //
}

// Emilio Nájera — June 27th, 2025