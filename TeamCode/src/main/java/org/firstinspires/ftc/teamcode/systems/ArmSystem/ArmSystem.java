package org.firstinspires.ftc.teamcode.systems.ArmSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.JointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.jointSubsystem.commands.JointPosition;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sliderSubsystem.commands.SliderPosition;

public class ArmSystem {
    // Subsystems
    SliderSubsystem sliderSubsystem;
    JointSubsystem jointSubsystem;

    // Setup code //

    // Enum to represent the individual subsystems and, therefore, order in which they'll move
    private enum ArmSubsystem {
        sliderSubsystem, jointSubsystem
    }

    public static class ArmOrder {
        /* This class gets information regarding in what order will the subsystems move, saved
         * in the "first" and "second" variables respectively
         *
         * It could be replaced with a Java 17+ record, allowing for simpler and more direct syntax:
         * public record ArmOrder(ArmSubsystem first, ArmSubsystem second) {}
         *
        */

        private final ArmSubsystem first;
        private final ArmSubsystem second;

        private ArmOrder(ArmSubsystem first, ArmSubsystem second) {
            this.first = first;
            this.second = second;
        }

        private ArmSubsystem getFirst() {
            return first;
        }

        private ArmSubsystem getSecond() {
            return second;
        }
    }

    public enum ArmOrderOptions {
        // These are the available orders in which the arm can move
        JS(new ArmOrder(
                // In this first option, the joint moves first and then does the slider
                ArmSubsystem.jointSubsystem, ArmSubsystem.sliderSubsystem
        )),
        SJ(new ArmOrder(
                // In this second option, the slider does the work first followed by the joint
                ArmSubsystem.sliderSubsystem, ArmSubsystem.jointSubsystem
        ));

        // Constructor and additional code //

        // The order variable is necessary because the chosen order must be stored somewhere
        // This variable can later be returned through getOrder()
        private final ArmOrder order;

        ArmOrderOptions(ArmOrder order) {
            this.order = order;
        }

        public ArmOrder getOrder() {
            return order;
        }
    }

    // The following group of enums & classes allow me to declare certain positions for both subsystems
    // to simply try and reach them

    public static class Position {
        /*
         * This method could be replaced with a Java 17+ record for more simplicity:
         * public record Position(double sliderDisplacement, double jointAngle) {}
         *
         */
        private final double sliderDisplacement;
        private final double jointAngle;

        public Position(double sliderDisplacement, double jointAngle) {
            this.sliderDisplacement = sliderDisplacement;
            this.jointAngle = jointAngle;
        }

        public double getSliderDisplacement() {
            return sliderDisplacement;
        }

        public double getJointAngle() {
            return jointAngle;
        }
    }

    public enum ArmPoseOptions {
        // Defined positions to which the arm can move
        QUESADILLA(new Position(0, 0)),
        HIGH_BASKET(new Position(200, 30));

        // Constructor and method to get the position
        public final Position position;

        ArmPoseOptions(Position position) {
            this.position = position;
        }

        public Position getPosition() {
            return position;
        }
    }

    // Constructor //
    public ArmSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Assigning subsystems using hardwareMap and telemetry
        sliderSubsystem = new SliderSubsystem(hardwareMap, telemetry);
        jointSubsystem = new JointSubsystem(hardwareMap, telemetry);
    }

    // Functional code //

    // So, essentially, this function determines dynamically which subsystem and what action to perform
    private Command getCommandForSubsystem(Position pose, ArmSubsystem member) {
        // Depending on what subsystem we are referring to, the switch case will perform an action on
        // either the slider or joint subsystems.
        switch (member) {
            // This performed action is going to be the passed pose's slider displacement or joint angle,
            // depending of course on which subsystem was selected through the switch case

            //case sliderSubsystem -> sliderSubsystem.setDistance(pose.sliderDisplacement());
            case sliderSubsystem:
                return new SliderPosition(sliderSubsystem, pose.getSliderDisplacement());

            //case jointSubsystem -> jointSubsystem.setAngle(pose.jointAngle());
            case jointSubsystem:
                return new JointPosition(jointSubsystem, pose.getJointAngle());


            default:
                throw new IllegalArgumentException("Unknown subsystem: " + member);
        }
    }

    // This is the master command of all the system.
    // It takes a pose parameter, which refers to the position subsystems should reach
    // And an order parameter as well, which defines the order in which subsystems will move

    /*
     * Let's say for example I wanted to get my arm in the QUESADILLA position, then I'd do
     * setPose(ArmPoseOptions.QUESADILLA, ArmOrderOptions.SJ);
     *
     * This is because first we are getting the position values from QUESADILLA and then
     * the order values from SJ through their getPosition() & getOrder() methods
     *
     */

    public SequentialCommandGroup setPose(ArmPoseOptions pose, ArmOrderOptions order) {
        return new SequentialCommandGroup(
                getCommandForSubsystem(pose.getPosition(), order.getOrder().getFirst()),
                getCommandForSubsystem(pose.getPosition(), order.getOrder().getSecond())
        );
    }
}

// Emilio Nájera — May-31st-2025