package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// ! F Constants stands for FollowerConstants
public class FConstants {
    static {
        // Selecting our localizer
        FollowerConstants.localizers = Localizers.OTOS;

        // Setting up motors. They should match the mecanum's constant values
        FollowerConstants.leftFrontMotorName = "mecanumFrontLeftMotor";
        FollowerConstants.leftRearMotorName = "mecanumBackLeftMotor";
        FollowerConstants.rightFrontMotorName = "mecanumFrontRightMotor";
        FollowerConstants.rightRearMotorName = "mecanumBackRightMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // Total weight of the robot
        FollowerConstants.mass = 12; // in KG

        // Forward velocity of the robot:
        FollowerConstants.xMovement = 57.8741;

        // Lateral velocity of the robot
        FollowerConstants.yMovement = 52.295;

        // Acceleration of the forward drivetrain when power is cut. It is in inches/second^2
        FollowerConstants.forwardZeroPowerAcceleration = -41.278;

        // Acceleration of the lateral drivetrain when power is cut. It is in inches/second^2
        FollowerConstants.lateralZeroPowerAcceleration = -59.7819;

        /*
         * PedroPathing allows us to choose between one or two PID controllers for each correction type
         *
         * In case of choosing two PIDs, the first one is responsible for handling larger errors,
         * while the second one takes charge of solving smaller corrections
         *
         * The main PID basically moves the error to the range of the secondary PID, which takes it
         * and minimizes oscillations
         *
         * To enable the two-PID system, enable true to the useSecondary[TYPE]PID we'd like to use
         *
         * Documentation link: https://pedropathing.com/pidf/intro.html
         */

        // Translational PID — maintains the robot on its path
        // To tune: https://pedropathing.com/pidf/translational.html
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
            // In case of wanting to use two PIDs, just set the following value to true
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);

        // Heading PID — controls rotational alignment
        // To tune: https://pedropathing.com/pidf/heading.html
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
            // In case of wanting to use two PIDs, just set the following value to true
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0);

        // Drive PID — manages acceleration and braking along paths
        // To tune: https://pedropathing.com/pidf/drive.html
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0);
            // In case of wanting to use two PIDs, just set the following value to true
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0);

        // Centripetal Force Correction — corrects deviations on curved paths
        // To tune: https://pedropathing.com/pidf/centripetal.html
        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}