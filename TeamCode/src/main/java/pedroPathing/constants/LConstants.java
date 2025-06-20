package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* ! L Constants stands for LocalizerConstants
 *  This acts as a method of configuring/updating the Localizer without direct access to it
 *
 *  Refer to this link for additional info: https://pedropathing.com/localization/otos.html
 */
public class LConstants {
    static {
        // Basic OTOS sensor configuration
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "otos"; // name of the OTOS declared in the Control Hub

        // Setting up units in which PedroPathing will measure
        OTOSConstants.linearUnit = DistanceUnit.CM; // original value is in inches
        OTOSConstants.angleUnit = AngleUnit.RADIANS;

        // Position of the sensor relative to the center of the robot, seen from the top
        // and having the front being the top side of the robot chassis
        /* @Parameters
            x: Forward/backward offset in inches. Forward is a positive x value.
            y: Left/right offset in inches. Left is a positive y value
            h: π/2 is facing forward. Clockwise rotation = negative
        * */
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);

        // ! Localizer tuner
        // Refer to the following link for additional info:
        // https://pedropathing.com/localization/otos.html#_2-localizer-tuning

        // This first value represents the forward/lateral tuning of the OTOS lectures
        OTOSConstants.linearScalar = 1.0;

        // This second value represents the rotational tuning of the OTOS lectures
        OTOSConstants.angularScalar = 1.0;
    }
}