package org.firstinspires.ftc.teamcode;


import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.JointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.commands.JointPosition;
import org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.commands.SliderPosition;
import org.firstinspires.ftc.teamcode.systems.ArmSystem.ArmSystem;
import org.firstinspires.ftc.teamcode.util.JoystickSupplier;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops
@TeleOp(name = "CMD", group = "Op mode")
public class CMDOpMode extends CommandOpMode {
    // Declaring input systems //
    // gamepad1 comes with FTC SDK. To declare a more maneuverable version, I declared my own controller through GamepadEx
    // With this in mind, it GamepadEx takes one parameter, which is an already-declared FTC SDK controller (gamepad1)
    GamepadEx controller;


    // Declaring systems and subsystems //
    MecanumSubsystem mecanumSubsystem;

    // ! These are test subsystems for me to try their functionality individually
    JointSubsystem testJoint;
    SliderSubsystem testSlider;
    ArmSystem arm;

    @Override
    public void initialize() {
        // Here, declare code to be executed right after pressing the INIT button
        // Including, for example, declaration of subsystems, configuring the IMU, etc.

        // Configuring control bindings
        controller = new GamepadEx(gamepad1);
        configureBindings();

        // Declaring subsystems & their default commands //
        // Default commands are executed every time another command is not being executed
        // This makes it perfect for tasks like setting up the drive train and that stuff lmao
        mecanumSubsystem = new MecanumSubsystem(hardwareMap, telemetry);
        mecanumSubsystem.setDefaultCommand(
                new DriveCommand(
                        mecanumSubsystem,
                        new JoystickSupplier(() -> controller.getLeftX(), () -> controller.getLeftY()),
                        () -> controller.getRightX()
                )
        );

        testJoint = new JointSubsystem(hardwareMap, telemetry);
        testSlider = new SliderSubsystem(hardwareMap, telemetry);

        arm = new ArmSystem(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize();

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart();

        // Run the scheduler
        while (opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run();

            // Constantly updating the telemetry
            telemetry.update();
        }
    }

    private void configureBindings() {
        // All control bindings are declared here
        new GamepadButton(controller, GamepadKeys.Button.A)
                .whileHeld(new JointPosition(testJoint, 35));

        new GamepadButton(controller, GamepadKeys.Button.B)
                .whenPressed(new SliderPosition(testSlider, 4))
                .whenReleased(new SliderPosition(testSlider, 10));

        new GamepadButton(controller, GamepadKeys.Button.X)
                .whenPressed(arm.setPose(ArmSystem.ArmPoseOptions.QUESADILLA, ArmSystem.ArmOrderOptions.SJ));
        new GamepadButton(controller, GamepadKeys.Button.Y)
                .whenPressed(arm.setPose(ArmSystem.ArmPoseOptions.HIGH_BASKET, ArmSystem.ArmOrderOptions.SJ));
    }
}

// Emilio Nájera, May—29th—2025