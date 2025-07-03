package org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.misc.MecanumConstants;
import org.firstinspires.ftc.teamcode.util.JoystickSupplier;

public class MecanumSubsystem extends SubsystemBase {

    // Useful variables
    HardwareMap hardwareMap;
    Telemetry telemetry;


    // Declaring sensors & extra devices
    SparkFunOTOS otos; // OTOS is an accelerometer
    IMU imu; // IMU returns direction, perfect for odometry


    // Declaring motors
    MotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    MotorEx[] motors = {};

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

        // Setting up sensors
        otos = hardwareMap.get(SparkFunOTOS.class, MecanumConstants.Setup.otosId);

        imu = hardwareMap.get(IMU.class, MecanumConstants.Setup.imuId);
        imu.resetYaw();

        // Setting up motors
        frontLeftMotor = new MotorEx(hardwareMap, MecanumConstants.Setup.frontLeftMotorId);
        frontRightMotor = new MotorEx(hardwareMap, MecanumConstants.Setup.frontRightMotorId);
        backLeftMotor = new MotorEx(hardwareMap, MecanumConstants.Setup.backLeftMotorId);
        backRightMotor = new MotorEx(hardwareMap, MecanumConstants.Setup.backRightMotorId);

        motors = new MotorEx[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        motorSetup();
    }


    @Override
    public void periodic() {
        // This function will be called every time the Scheduler is ran
        // todo: create a pose estimator using otos
        System.out.println("cutie pie");
    }

    // Functions //
    public void drive(JoystickSupplier leftJoystick, Supplier<Double> rightJoystick) {
        // Process to follow:
        // Getting joystick input -> Convert it to ChassisSpeeds -> Convert it to kinematics
        motorSetup();

        // Chassis velocities
        double strafeVelocityX = leftJoystick.getX().get(); // Strafe velocity (front & back)
        double strafeVelocityY = leftJoystick.getY().get(); // Strafe velocity (sideways)
        double angularVelocity = rightJoystick.get(); // Angular velocity

        // Creating a field-relative chassis speeds from the given data
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                strafeVelocityX, strafeVelocityY, angularVelocity, getHeading()
        );

        // Converting chassisSpeeds object to actual wheel velocities
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.normalize(1.0);

        // Assuming it is the bare motor with simply two stacked 3:1 reductions, it would be
        // RPM = 6000, Encoder Ticks per rev (Output Shaft) = 28 ->
        // Ticks/Second = (RPM/60) * Ticks per rev
        // Ticks/Second = 2800
        // This is because setVelocity() uses Encoder ticks per second, so we simply convert to this unit
        double desiredTicksPerSecond = MecanumConstants.Values.desiredTicksPerSecond;

        // The following if statement sets a dead-zone to avoid joystick drift issues
        if (0.3 > Math.abs(strafeVelocityX) && 0.3 > Math.abs(strafeVelocityY) && 0.3 > Math.abs(angularVelocity)) {
            stopMotors();
        } else {
            // Motor output. It takes the velocity in ticks per second
            // From meters/sec -> ticks/sec:
            // ticks/meter: (ticksPerRev * gearRatio) / wheelCircumference
            // ticks/second: ticks/meter * metersPerSec
            frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * desiredTicksPerSecond);
            frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond * desiredTicksPerSecond);
            backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * desiredTicksPerSecond);
            backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond * desiredTicksPerSecond);
        }

        /* Temporary telemetry to debug effectively

        telemetry.addData("Front left motor velocity output * desiredTickOutput: ", wheelSpeeds.frontLeftMetersPerSecond * desiredTicksPerSecond);
        telemetry.addData("Front right motor velocity output * desiredTickOutput: ", wheelSpeeds.frontRightMetersPerSecond * desiredTicksPerSecond);
        telemetry.addData("Back left motor velocity output * desiredTickOutput: ", wheelSpeeds.rearLeftMetersPerSecond * desiredTicksPerSecond);
        telemetry.addData("Back right motor velocity output * desiredTickOutput: ", wheelSpeeds.rearRightMetersPerSecond * desiredTicksPerSecond);

        telemetry.update();*/
    }

    // Stops all motors
    public void stopMotors() {
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        backRightMotor.stopMotor();
    }

    // Resets the mecanum heading manually
    public void resetHeading() {
        imu.resetYaw();
    }

    // Returns the current mecanum heading reading
    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }


    // Setup //
    private void motorSetup() {
        // Setting motors dynamically through an array to make it more readable
        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            motor.resetEncoder();

            // Execution mode using PIDs
            motor.setRunMode(MotorEx.RunMode.VelocityControl);
            motor.setVeloCoefficients(0.5, 0.0, 0.0);
        }

        // Configures the left motor
        frontLeftMotor.setInverted(true);
        frontLeftMotor.setVeloCoefficients(0.1, 0.0, 0.0);
    }
}

// Emilio Nájera, May—29th—2025