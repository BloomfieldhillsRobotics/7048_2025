

package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class PurpleProtonRobot extends SubsystemGroup {
    public static final PurpleProtonRobot INSTANCE = new PurpleProtonRobot();

    public final Command intakeStop =
            new SequentialGroup(
                    Intake.INSTANCE.stop
            ).named("IntakeStop");
    public final Command intakeRun =
            new SequentialGroup(
                    Intake.INSTANCE.run
            ).named("IntakeStop");
    public final Command intake =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    Intake.INSTANCE.run
            ).named("Intake");
    public final Command shoot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.run,
                    Elevator.INSTANCE.up,
                    Intake.INSTANCE.stop
            ).named("Shoot");
    public final Command score =
            new SequentialGroup(
                    intake,
                    new Delay(2),
                    shoot
            ).named("Score");
    public final Command elevatorUp =
            new SequentialGroup(
                    Elevator.INSTANCE.up
            ).named("ElevatorUp");
    public final Command elevatorDown =
            new SequentialGroup(
                    Elevator.INSTANCE.down
            ).named("ElevatorDown");
}