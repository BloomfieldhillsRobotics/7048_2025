package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private MotorEx Intake = new MotorEx("Intake");


    public Command run = new SetPower(Intake,.8).requires(this);
    public Command stop = new SetPower(Intake,0).requires(this);

    public Command runBackwards = new SetPower(Intake,-0.7).requires(this);
}