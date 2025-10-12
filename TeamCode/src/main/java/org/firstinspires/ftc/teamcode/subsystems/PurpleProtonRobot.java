

package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class PurpleProtonRobot extends SubsystemGroup {
    public static final PurpleProtonRobot INSTANCE = new PurpleProtonRobot();

    private PurpleProtonRobot() {
        super(
                Intake.INSTANCE,
                FlyWheel.INSTANCE,
                Elevator.INSTANCE
        );
    }
    public final Command intakeStop =
            new SequentialGroup(
                    Intake.INSTANCE.stop
            ).named("IntakeStop");
    public final Command intakeRun =
            new SequentialGroup(
                    Intake.INSTANCE.run
            ).named("IntakeRun");
    public final Command intake =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    Intake.INSTANCE.run
            ).named("Intake");
    public final Command shoot =
            new SequentialGroup(
                    Intake.INSTANCE.stop,
                    FlyWheel.INSTANCE.run,
                    Elevator.INSTANCE.up,
                    new Delay(4),
                    FlyWheel.INSTANCE.stop
            ).named("Shoot");

    public final Command FlyWheelRun =
            new SequentialGroup(
                    FlyWheel.INSTANCE.run
            ).named("FlyWheelRun");

    public final Command FlyWheelStop =
            new SequentialGroup(
                    FlyWheel.INSTANCE.stop
            ).named("FlyWheelStop");

    public final Command score =
            new SequentialGroup(
                    intake,
                    new Delay(2),
                    intakeStop,
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