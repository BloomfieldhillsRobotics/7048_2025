package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class BaseHook implements Subsystem {
    public static final BaseHook INSTANCE = new BaseHook();

    private BaseHook() { }

    public void BaseHookLock() {
        plate.setPosition(0.5);
    }

    private ServoEx plate = new ServoEx("plate");
    public Command lock = new SetPosition(plate, 0.5).requires(this);
    public Command unlock = new SetPosition(plate, 1).requires(this);
}