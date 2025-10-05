package org.firstinspires.ftc.teamcode;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private MotorEx Intake = new MotorEx("Intake");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command run = new SetPower(Intake,-1).requires(this);
    public Command stop = new SetPower(Intake,0).requires(this);

    @Override
    public void periodic() {
        Intake.setPower(controlSystem.calculate(Intake.getState()));
    }
}