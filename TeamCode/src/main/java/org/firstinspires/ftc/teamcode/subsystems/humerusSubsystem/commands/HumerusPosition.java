package org.firstinspires.ftc.teamcode.subsystems.humerusSubsystem.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.humerusSubsystem.HumerusSubsystem;

public class HumerusPosition extends CommandBase {
    // Declaring useful variables
    HumerusSubsystem humerusSubsystem;
    double angle;

    // Constructor & Functional code //
    public HumerusPosition(HumerusSubsystem humerusSubsystem, double angle) {
        // Assigning values to the class variables using passed arguments
        this.humerusSubsystem = humerusSubsystem;
        this.angle = angle;

        // addRequirements() tells the Scheduler that this subsystem is being used in this command
        addRequirements(humerusSubsystem);
    }

    @Override
    public void execute() {
        // Calling the method .setAngle() coming directly from the humerusSubsystem class
        humerusSubsystem.setAngle(angle);
    }
}

// Emilio Nájera — June 27th, 2025