package org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.misc.SliderConstants;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.misc.SliderMath;
import org.firstinspires.ftc.teamcode.util.MathUtils;


public class SliderSubsystem extends SubsystemBase {
    // Declaring useful variables
    HardwareMap hardwareMap;
    Telemetry telemetry;
    SliderMath sliderMath = new SliderMath(
            SliderConstants.MotorProperties.ticksPerRevolution,
            SliderConstants.MotorProperties.linearTravelPerRevolution,
            SliderConstants.MotorProperties.gearRatio
        );

    // Declaring motors & sensors
    MotorEx rightSliderMotor, leftSliderMotor;
    MotorGroup sliderMotors;

    // Constructor //
    public SliderSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Assigning values to useful variables
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Setting up motors
        // Reduction can be applied directly, with a 5.2:1 planetary box -> Motor.GoBILDA.RPM_1150
        // Or, with a 3.7:1 planetary box -> Motor.GoBILDA.RPM_1620
        // But I decided to apply it later on as part of the constants.
        rightSliderMotor = new MotorEx(hardwareMap, SliderConstants.Setup.rightSliderMotorId, Motor.GoBILDA.BARE);
        leftSliderMotor = new MotorEx(hardwareMap, SliderConstants.Setup.leftSliderMotorId, Motor.GoBILDA.BARE);

        sliderMotors = new MotorGroup(rightSliderMotor, leftSliderMotor);
        motorSetup();
    }

    // Functional code //

    // This function essentially takes a distance in centimeters and and calls the motor to go for it
    // It clamps the argument not to break the arm
    public void setDistance(double distance) {
        // Clamping the distance to ensure it is within the limits
        int targetDistance = MathUtils.clamp(
                sliderMath.toTicks(distance),
                SliderConstants.MeasureLimits.minTicksAllowed,
                SliderConstants.MeasureLimits.maxTicksAllowed);

        telemetry.addData("target distance form slider: ", targetDistance);
        telemetry.addData("encoder lecture: ", rightSliderMotor.encoder.getPosition());

        // Calling the clamped distance and reaching that position
        sliderMotors.setTargetPosition(targetDistance);
    }

    // Setup code //
    private void motorSetup() {
        // Inverted motor
        leftSliderMotor.setInverted(true);

        // Setting motors dynamically through an the sliderMotors motorGroup
        sliderMotors.resetEncoder();
        sliderMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        sliderMotors.setRunMode(Motor.RunMode.PositionControl);
        sliderMotors.setPositionCoefficient(1.0); // todo: tune this value like a regular PID
    }
}

// Emilio Nájera — May-30th-2025
