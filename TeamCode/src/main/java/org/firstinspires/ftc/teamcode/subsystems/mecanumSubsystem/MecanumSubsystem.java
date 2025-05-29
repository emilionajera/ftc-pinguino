package org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Rotation2dExtKt;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.JoystickSupplier;

import kotlin.Unit;

public class MecanumSubsystem extends SubsystemBase {

    // Useful variables
    HardwareMap hardwareMap;
    Telemetry telemetry;


    // Declaring sensors & extra devices
    SparkFunOTOS otos; // OTOS is an accelerometer
    IMU imu; // IMU returns direction, perfect for odometry


    // Declaring motors
    MotorEx frontLeftMotor = new MotorEx(hardwareMap, "motorOne"); // todo: check these ids
    MotorEx frontRightMotor = new MotorEx(hardwareMap, "motorTwo");
    MotorEx backLeftMotor = new MotorEx(hardwareMap, "motorThree");
    MotorEx backRightMotor = new MotorEx(hardwareMap, "motorFour");
    MotorEx[] motors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor}; // todo: Utilizar un motorGroup para el elevador y demás subsistemas

    // Setting up motor locations relative to the robot center and kinematics object
    Translation2d frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d backRightLocation =
            new Translation2d(-0.381, -0.381);

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation
    );



    // Functional code //

    public MecanumSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Assigning variables to the class' equivalents
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Setting up motors and sensors
        motorSetup();
        imu.resetYaw();
    }


    @Override
    public void periodic() {
        // This function will be called every time the Scheduler is ran
        // todo: create a pose estimator
        System.out.println("cutie pie");
    }

    // Functions //
    public void drive(JoystickSupplier leftJoystick, Supplier<Double> rightJoystick) {
        // Process to follow:
        // Getting joystick input -> Convert it to ChassisSpeeds -> Convert it to kinematics

        // Chassis velocities
        double strafeVelocityX = leftJoystick.x().get(); // Strafe velocity (front & back)
        double strafeVelocityY = leftJoystick.y().get(); // Strafe velocity (sideways)
        double angularVelocity = rightJoystick.get(); // Angular velocity

        // Creating a field-relative chassis speeds from the given data
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                strafeVelocityX, strafeVelocityY, angularVelocity, getHeading()
        );

        // Converting chassisSpeeds object to actual wheel velocities
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        // Motor output
        frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond);
        backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond);

        // todo: scale powers
    }

    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    // Setup //

    private void motorSetup() {
        // Setting motors dynamically through an array without depending
        for (MotorEx motor : motors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }
}

// Emilio Nájera, May—29th—2025