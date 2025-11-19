

package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class TestBed extends SubsystemGroup {
    public static final TestBed INSTANCE = new TestBed();

    private TestBed() {
        super(
                FlyWheel.INSTANCE
        );
    }

    public final Command FlyWheelStop = new SequentialGroup(
                    FlyWheel.INSTANCE.stop
            ).named("FlyWheelStop");
    public final Command FlyWheelLongShot = new SequentialGroup(
                    FlyWheel.INSTANCE.longshot
            ).named("FlyWheelLongShot");
    public final Command FlyWheelSuperLongShot = new SequentialGroup(
            FlyWheel.INSTANCE.superlongshot
    ).named("FlyWheelSuperLongShot");
    public final Command FlyWheelShortShot = new SequentialGroup(
            FlyWheel.INSTANCE.shortshot
    ).named("FlyWheelLongShot");
}