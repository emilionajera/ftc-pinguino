package org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.subsystems.mecanumSubsystem.MecanumSubsystem;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.util.JoystickSupplier;

public class DriveCommand extends CommandBase {
    // Declaring useful variables
    MecanumSubsystem mecanumSubsystem;
    JoystickSupplier leftJoystick;
    Supplier<Double> rightJoystick;


    public DriveCommand(MecanumSubsystem mecanumSubsystem, JoystickSupplier leftJoystick,
                        Supplier<Double> rightJoystick) {
        // Assigning values to class variables
        this.mecanumSubsystem = mecanumSubsystem;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;

        // addRequirements() tells the Scheduler that the Mecanum Subsystem is being used in this command
        addRequirements(this.mecanumSubsystem);
    }

    @Override
    public void execute() {
        mecanumSubsystem.drive(leftJoystick, rightJoystick);
    }
}

// Emilio Nájera, May—29th—2025