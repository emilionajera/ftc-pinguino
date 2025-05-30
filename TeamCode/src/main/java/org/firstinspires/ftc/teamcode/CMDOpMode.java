package org.firstinspires.ftc.teamcode;


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
import org.firstinspires.ftc.teamcode.util.JoystickSupplier;

// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops
@TeleOp(name = "CMD", group = "Op mode")
public class CMDOpMode extends CommandOpMode {
    // Declaring input systems //
    // gamepad1 comes with FTC SDK. To declare a more maneuverable version, I declared my own controller through GamepadEx
    // With this in mind, it GamepadEx takes one parameter, which is an already-declared FTC SDK controller (gamepad1)
    GamepadEx controller = new GamepadEx(gamepad1);


    // Declaring systems and subsystems //
    MecanumSubsystem mecanumSubsystem;
    JointSubsystem testJoint;

    @Override
    public void initialize() {
        // Here, declare code to be executed right after pressing the INIT button
        // Including, for example, declaration of subsystems, configuring the IMU, etc.

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

        // Configuring control bindings
        configureBindings();
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
        }

    }

    private void configureBindings() {
        // All control bindings are declared here
        new GamepadButton(controller, GamepadKeys.Button.A)
                .whenPressed(new JointPosition(testJoint, 35));
    }
}

// Emilio Nájera, May—29th—2025