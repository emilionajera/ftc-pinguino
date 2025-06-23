package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

// Tools to generate paths
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.JointSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

// ! Use the following visualizer to visually track and define auto routes:
// https://visualizer.pedropathing.com/

// For an autonomous example, check out the following documentation:
// https://pedropathing.com/pedro/examples/auto.html

@Autonomous(name="First auto test", group="Auto testing")
public class AutoTest extends OpMode {

    // Test subsystems
    JointSubsystem testJoint;

    /* Declaring the follower object. It is really important to declare this object or else
     * the auto won't work at all
     *
     * Several useful methods, including: —————————————————————————————
     *
     * .turnToDegrees(deg) — turns to a specific heading in degrees
     *
     * Refer to this documentation for more info: https://shorturl.at/jY7LN
     */
    private Follower follower;

    // Declaring a timer to limit the time the auto runs through
    Timer pathTimer;

    // Declaring poses the robot should reach
    private final Pose startingPose = new Pose(8, 112, Math.toRadians(270));
    private final Pose middlePose = new Pose(66, 135, Math.toRadians(180));
    private final Pose endPose = new Pose(105, 112, Math.toRadians(270));


    // Declaring paths for the robot to follow across
    // Paths (to build pathChains) -> PathChains (to follow) -> PathBuilder (to build path chains)
    private Path testPath;
    private PathChain testPathChain;


    // The variable pathState tracks the robot's progress through the paths it must follow
    // Basically, it is a counter variable
    private int pathState;





    // ! Initialization ! //

    // Refer to https://shorturl.at/P6emO for extra information about initializing
    @Override
    public void init() {
        // init() phase sets up timers and paths (through buildPaths() method)

        // The following line of code ensures FollowerConstants and LocalizerConstants are
        // updated with user-set values.
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startingPose); // Setting up the starting pose

        pathTimer = new Timer();
        buildPaths();
    }

    @Override
    public void loop() {
        // loop() ensures continuous updates during autonomous after pressing Play
        follower.update(); // Loops the movement of the robot
        autonomousPathUpdate(); // Constantly checks the state the robot is in

        // Telemetry
        telemetry.addData("Current path state", pathState);
        telemetry.addData("Position", follower.getPose().toString());

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();
    }

    @Override
    public void start() {
        // start() is called once at the start of the OpMode
        setPathState(0);
    }





    // ! Functional code ! //

    /*
     * The following method takes pathState and decides what to do according to the current value
     * Each state in the autonomousPathUpdate() method corresponds to a specific action or movement
     * The robot transitions between states when certain conditions are met
     *
     * Refer to https://shorturl.at/J3h5f for documentation
     */
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0 -> {
                follower.followPath(testPath);
                setPathState(1);
            }

            case 1 -> {
                /*
                 * The if statement allows me to verify that the robot is in fact reaching an
                 * x, y position before proceeding to the next states, ensuring no space
                 * for premature transitions
                 *
                 * */
                if (!follower.isBusy()) {
                    follower.followPath(testPathChain);
                    setPathState(2);
                }
            }

            case 2 -> {
                if (!follower.isBusy()) {
                    setPathState(-1); // End of the autonomous routine
                }
            }

        }
    }

    // The following method allows me to change to different states through integers
    public void setPathState(int desiredState) {
        pathState = desiredState;
        pathTimer.resetTimer();
    }





    // ! Setup code ! //

    // The creation of this method allows for the paths to be ready before starting the autonomous
    // It is called in the init() / initialize() method
    public void buildPaths() {
        /*
        * The Path class handles information regarding the actual path the Follower will follow
        * It has several useful methods like: —————————————————————————————
        *
        * zeroPowerAccelerationMultiplier() — speed at which the robot decelerates at the end
        * of paths
        *
        * pathEndTimeoutConstraint() — When the path is considered at its end parametrically,
        * the Follower has this many milliseconds to correct by default
        *
        * Refer to this documentation for more info: https://shorturl.at/b6Vcr
        *
        * */

        testPath = new Path(new BezierLine(new Point(startingPose), new Point(middlePose)));
        testPath.setLinearHeadingInterpolation(startingPose.getHeading(), middlePose.getHeading());

        /*
        * The PathChain class chains together multiple paths into a larger collection of paths,
        * avoiding having to execute them individually.
        *
        * The PathBuilder allows us to easily create create PathChains, so we don't have
         * to individually create Path instances to create a PathChain
        *
        * This allows the paths to execute continuously without any pauses in between
        *
        * Similar to PathPlanner's "Auto Builder" feature, where paths are chained together too
        *
        * Useful methods: —————————————————————————————
        *
        * getPath() — returns the path being ran at the moment
        *
        * setLinearHeadingInterpolation() — describes the way the heading will change
        * from path point to path point
        *
        * Refer to this documentation for more info: https://shorturl.at/xvLP1
        *
        * */

        testPathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(middlePose), new Point(endPose)))
                .setLinearHeadingInterpolation(middlePose.getHeading(), endPose.getHeading())
                .build();

    }

}

// Emilio Nájera — June 21st, 2025