package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

// ! Código para OpMode, donde varios procesos pueden ser ejecutados a la vez
// En un LinearOpMode, los procesos son ejecutados paso a paso, mientras que un OpMode, son ejecutados en un bucle
@TeleOp(name = "CMD", group = "Op mode")
public class CMDOpMode extends CommandOpMode {
    // Declaring input variables //
    GamepadEx gamepad = new GamepadEx(HardwareMap.gamepad);


    // Declaring subsystems


    @Override
    public void initialize() {
        // Aquí va el código que se ejecuta justo después de presionar el botón INIT
        // Como, por ejemplo, declarar de subsistemas, setear el IMU, etc.

        configureBindings();
    }

    @Override
    public void runOpMode() {
        // Código que se tiene que ejecutar constantemente
        initialize();
        CommandScheduler.getInstance().run();

        // Pausa el OpMode hasta que el botón de START sea presionado en la Driver Hub
        waitForStart();

    }

    private void configureBindings() {
        // Aquí dentro se declaran todos los bindings de controles
    }
}
