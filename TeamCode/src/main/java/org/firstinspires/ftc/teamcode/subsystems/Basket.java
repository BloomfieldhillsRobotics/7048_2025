package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Basket implements Subsystem {
    public static final Basket INSTANCE = new Basket();

    private Basket() { }
    private ServoEx basketServo = new ServoEx("BasketServo");
    public Command up = new SetPosition(basketServo, 0.4).requires(this);
    public Command down = new SetPosition(basketServo, 0.75).requires(this);
}