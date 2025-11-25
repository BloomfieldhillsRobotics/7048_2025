

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.TeleOpProgram;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class PurpleProtonRobot extends SubsystemGroup {
    public static final PurpleProtonRobot INSTANCE = new PurpleProtonRobot();

    private PurpleProtonRobot() {
        super(
                Intake.INSTANCE,
                FlyWheel.INSTANCE,
                Elevator.INSTANCE,
                Basket.INSTANCE,
                Lifter.INSTANCE,
                BaseHook.INSTANCE
        );
    }
    public static double dynamicFlywheelSpeed = 1600;

    public final Command autoshot =
            new SequentialGroup(
                    new InstantCommand(() -> {
                        FlyWheel.INSTANCE.setTargetSpeed(dynamicFlywheelSpeed);
                    }));
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
    public final Command LifterStop =
            new SequentialGroup(
                    Lifter.INSTANCE.stop
            ).named("LifterStop");
    public final Command LifterRun =
            new SequentialGroup(
                    Lifter.INSTANCE.run
            ).named("LifterRun");
    public final Command LifterRunBackwards =
            new SequentialGroup(
                    Lifter.INSTANCE.runBackwards
            ).named("LifterRunBackwards");
    public final Command AutoShoot =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    new Delay(.2),
                    autoshot,
                    new Delay(.5),
                    Elevator.INSTANCE.up,
                    new Delay(.5),
                    Elevator.INSTANCE.down,
                    new Delay(.2),
                    FlyWheel.INSTANCE.stop
            ).named("AutoShoot");

    public final Command ShortShot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.shortshot,
                    new Delay(.5),
                    Elevator.INSTANCE.up,
                    new Delay(.5),
                    Elevator.INSTANCE.down,
                    new Delay(.5),
                    FlyWheel.INSTANCE.stop
            ).named("ShortShot");
    public final Command FireNote =
            new SequentialGroup(
                    new Delay(.5), // Give flywheel time to recover speed
                    Elevator.INSTANCE.up,
                    new Delay(.8),
                    Elevator.INSTANCE.down
            ).named("FireNote");

    // We keep the FlyWheelStop command as a convenient shortcut
    public final Command FlyWheelStop =
            new SequentialGroup(
                    FlyWheel.INSTANCE.stop
            ).named("FlyWheelStop");
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

    public final Command score =
            new SequentialGroup(
                    IntakeSeq,
                    new Delay(2),
                    IntakeStop,
                    AutoShoot
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
    public final Command shortshot =
            new SequentialGroup(
                    FlyWheel.INSTANCE.shortshot
            ).named("ShortShot");
    public final Command runIntakeBackwards =
            new SequentialGroup(
                    Intake.INSTANCE.runBackwards
            ).named("runIntakeBackwards");
    public final Command BasketDrop =
            new SequentialGroup(
                    Elevator.INSTANCE.down,
                    Basket.INSTANCE.down,
                    FlyWheel.INSTANCE.backwards,
                    new Delay(.7),
                    Basket.INSTANCE.up,
                    FlyWheel.INSTANCE.stop
            ).named("BasketDrop");
    public final Command BasketUp =
            new SequentialGroup(
                    Basket.INSTANCE.up,
                    FlyWheel.INSTANCE.stop
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
            ).named("AutoGPP3LongShot");
    public final Command AutoPPG3ShortShot =
            new SequentialGroup(
                    IntakeSeq,
                    ShortShot,
                    IntakeSeq,
                    ShortShot,
                    BasketDrop,
                    ShortShot
            ).named("AutoPPG3ShortShot");
    public final Command AutoPGP3ShortShot =
            new SequentialGroup(
                    IntakeSeq,
                    ShortShot,
                    BasketDrop,
                    ShortShot,
                    IntakeSeq,
                    ShortShot
            ).named("AutoPGP3ShortShot");
    public final Command AutoGPP3ShortShot =
            new SequentialGroup(
                    BasketDrop,
                    ShortShot,
                    IntakeSeq,
                    ShortShot,
                    IntakeSeq,
                    ShortShot
            ).named("AutoGPP3ShortShot");
    public final Command Auto3LongShot =
            new SequentialGroup(
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot,
                    IntakeSeq,
                    LongShot
            ).named("Auto3LongShot");
    public final Command Auto3ShortShot =
            new SequentialGroup(
                    IntakeSeq,
                    ShortShot,
                    IntakeSeq,
                    ShortShot,
                    IntakeSeq,
                    ShortShot
            ).named("Auto3ShortShot");

}