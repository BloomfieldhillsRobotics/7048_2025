package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Elevator implements Subsystem {
    public static final Elevator INSTANCE = new Elevator();

    private Elevator() { }
    private ServoEx liftServo = new ServoEx("LiftServo");
    public Command up = new SetPosition(liftServo, 0.1).requires(this);
    public Command down = new SetPosition(liftServo, 0.9).requires(this);
}