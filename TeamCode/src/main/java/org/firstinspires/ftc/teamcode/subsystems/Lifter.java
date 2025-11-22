package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Lifter implements Subsystem {
    public static final Lifter INSTANCE = new Lifter();
    private Lifter() { }

    private MotorEx Lifter = new MotorEx("odometer2");


    public Command run = new SetPower(Lifter,1).requires(this);
    public Command stop = new SetPower(Lifter,0).requires(this);

    public Command runBackwards = new SetPower(Lifter,-0.5).requires(this);
}