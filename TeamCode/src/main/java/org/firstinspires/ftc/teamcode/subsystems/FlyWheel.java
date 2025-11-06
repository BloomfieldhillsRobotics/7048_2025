package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class FlyWheel implements Subsystem {
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }
    MotorGroup FlyWheelGroup = new MotorGroup(
            new MotorEx("FlywheelRight"),
            new MotorEx("FlywheelLeft").reversed()
    );
    public Command superlongshot = new SetPower(FlyWheelGroup,1).requires(this);
    //public Command superlongshot = new Set
    public Command longshot = new SetPower(FlyWheelGroup,.75).requires(this);
    public Command shortshot = new SetPower(FlyWheelGroup,.6).requires(this);
    public Command stop = new SetPower(FlyWheelGroup,0).requires(this);
}