

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
                Elevator.INSTANCE,
                Basket.INSTANCE
        );
    }
    public final Command IntakeStop =
            new SequentialGroup(
                    Intake.INSTANCE.stop
            ).named("IntakeStop");
    public final Command IntakeRun =
            new SequentialGroup(
                    Intake.INSTANCE.run
            ).named("IntakeRun");
    public final Command IntakeSeq =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    Intake.INSTANCE.run,
                    new Delay(.5),
                    Intake.INSTANCE.stop
            ).named("IntakeSeq");
    public final Command Shoot =
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
                    FlyWheel.INSTANCE.shortshot,
                    new Delay(.5),
                    Elevator.INSTANCE.up,
                    new Delay(.8),
                    Elevator.INSTANCE.down,
                    new Delay(.5),
                    FlyWheel.INSTANCE.stop
            ).named("ShortShot");

    public final Command LongShot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.longshot,
                    new Delay(.5),
                    Elevator.INSTANCE.up,
                    new Delay(.8),
                    Elevator.INSTANCE.down,
                    new Delay(.5),
                    FlyWheel.INSTANCE.stop
            ).named("LongShot");

    public final Command FlyWheelStop =
            new SequentialGroup(
                    FlyWheel.INSTANCE.stop
            ).named("FlyWheelStop");

    public final Command score =
            new SequentialGroup(
                    IntakeSeq,
                    new Delay(2),
                    IntakeStop,
                    Shoot
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
    public final Command BasketDrop =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    Basket.INSTANCE.down,
                    new Delay(.5),
                    Basket.INSTANCE.up
            ).named("BasketDrop");
    public final Command BasketUp =
            new SequentialGroup(
                    Basket.INSTANCE.up
            ).named("BasketUp");
    public final Command AutoPPG3LongShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    BasketDrop,
                    LongShot
            ).named("AutoPPG3Shot");
    public final Command AutoPGP3LongShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    BasketDrop,
                    LongShot,
                    IntakeSeq,
                    LongShot
            ).named("AutoPGP3LongShot");
    public final Command AutoGPP3LongShot =
            new SequentialGroup(
                    BasketDrop,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot
            ).named("AutoPGP3LongShot");
    public final Command AutoPPG3ShortShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    BasketDrop,
                    LongShot
            ).named("AutoPPG3ShortShot");
    public final Command AutoPGP3ShortShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    BasketDrop,
                    LongShot,
                    IntakeSeq,
                    LongShot
            ).named("AutoPGP3ShortShot");
    public final Command AutoGPP3ShortShot =
            new SequentialGroup(
                    BasketDrop,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot
            ).named("AutoPGP3ShortShot");
    public final Command Auto3LongShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    BasketDrop,
                    LongShot
            ).named("Auto3LongShot");

}