package org.firstinspires.ftc.teamcode.subsystems.jointSubsystem;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc.JointConstants;
import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.misc.JointMath;
import org.firstinspires.ftc.teamcode.util.MathUtils;

public class JointSubsystem extends SubsystemBase {
    // Declaring useful variables
    HardwareMap hardwareMap;
    Telemetry telemetry;
    JointMath jointMath = new JointMath(JointConstants.MotorProperties.ticksFor90,
            JointConstants.MotorProperties.gearRatioReduction); // todo: check these numbers (declared in constants)

    // Declaring motors
    MotorEx rightJointMotor, leftJointMotor;
    MotorEx[] motors = {};


    // Constructor //
    public JointSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Setting up motors and motor group
        rightJointMotor = new MotorEx(hardwareMap, JointConstants.Setup.rightJointMotorId);
        leftJointMotor = new MotorEx(hardwareMap, JointConstants.Setup.leftJointMotorId);

        motors = new MotorEx[]{rightJointMotor, leftJointMotor};
        motorSetup();
    }


    // Functional code //

    // This function essentially takes an angle, and then, after clamping, makes the motors
    // reach that angle through setTargetPosition, which takes an argument in encoder ticks :)
    public void setAngle(double angle) {
        // Clamping the angle to ensure it is within the encoder's limits, preventing it from breaking
        int targetAngle = MathUtils.clamp(
                jointMath.toTicks(angle),
                JointConstants.MeasureLimits.minTicksAllowed,
                JointConstants.MeasureLimits.maxTicksAllowed);

        //telemetry.addData("joint target angle: ", targetAngle);

        // Calling the clamped angle and reaching that position
        rightJointMotor.setTargetPosition(targetAngle);
        leftJointMotor.setTargetPosition(targetAngle);
    }


    // Setup //

    private void motorSetup() {
        // Setting motors dynamically through an array to make it more readable
        for (MotorEx motor : motors) {
            motor.resetEncoder();
            motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

            // Execution mode using PIDs
            motor.setRunMode(MotorEx.RunMode.PositionControl);
            motor.setPositionCoefficient(1.0); // todo: tune this value like a regular PID
        }

        // Inverted motors
        leftJointMotor.setInverted(true);

    }
}

// Emilio Nájera, May—30th—2025