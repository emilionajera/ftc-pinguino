package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrivetrain extends SubsystemBase {

    // Useful variables
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Supplier<Double> xInput;
    Supplier<Double> yInput;

    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry,
                             Supplier<Double> xInput, Supplier<Double> yInput) {

        // Assigning variables to the class' equivalents
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.xInput = xInput;
        this.yInput = yInput;
    }

    @Override
    public void periodic() {
        // Esta función se llamará cada vez que sea ejecutado el scheduler
        System.out.println("cutie pie");
    }
}
