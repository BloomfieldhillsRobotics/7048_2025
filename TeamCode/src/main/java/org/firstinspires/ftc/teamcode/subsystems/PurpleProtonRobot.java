

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
                    Elevator.INSTANCE.down,
                    new Delay(.5),
                    Intake.INSTANCE.run,
                    new Delay(.5),
                    Intake.INSTANCE.stop,
                    new Delay(.5),
                    FlyWheel.INSTANCE.shortshot,
                    new Delay(.5),
                    Elevator.INSTANCE.up,
                    new Delay(.5),
                    FlyWheel.INSTANCE.stop,
                    new Delay(.5),
                    Elevator.INSTANCE.down
            ).named("Shoot");

    public final Command ShortShot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.shortshot
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
    public final Command superlongshot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.superlongshot
            ).named("Superlongshot");
    public final Command longshot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.longshot
            ).named("LongShot");
    public final Command runIntakeBackwards =
            new SequentialGroup(
                    Intake.INSTANCE.runBackwards
            ).named("runIntakeBackwards");

}