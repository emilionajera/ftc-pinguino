package org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.SliderSubsystem;

public class SliderPosition extends CommandBase {
    // Declaring useful variables
    SliderSubsystem sliderSubsystem;
    double distance;

    public SliderPosition(SliderSubsystem sliderSubsystem, double distance) {
        // Assigning values to the class variables using passed parameters
        this.sliderSubsystem = sliderSubsystem;
        this.distance = distance;

        // addRequirements() tells the scheduler that the subsystem is being used
        addRequirements(this.sliderSubsystem);
    }

    @Override
    public void execute() {
        sliderSubsystem.setDistance(distance);
    }
}

// Emilio Nájera — May-31st-2025