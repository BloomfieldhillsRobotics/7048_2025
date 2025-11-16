package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Set;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class FlyWheel implements Subsystem {
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }
    MotorGroup FlyWheelGroup = new MotorGroup(
            new MotorEx("FlywheelRight"),
            new MotorEx("FlywheelLeft").reversed()
    );
    public static double flywheelSpeed = 500.0;
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.0;


    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(kP, 0, kD)
            .basicFF(kV, kA, kS)
            .build();

    public Command superlongshot = new RunToVelocity(controlSystem, flywheelSpeed * 1.25).requires(this);
    public Command longshot = new RunToVelocity(controlSystem, flywheelSpeed).requires(this);
    public Command shortshot = new RunToVelocity(controlSystem, flywheelSpeed * .8).requires(this);
    public Command stop = new SetPower(FlyWheelGroup,0).requires(this);

    @Override
    public void periodic() {
        FlyWheelGroup.setPower(controlSystem.calculate(FlyWheelGroup.getState()));
        ActiveOpMode.telemetry().addData("Flywheel State", FlyWheelGroup.getState());
        ActiveOpMode.telemetry().addData("Flywheel Speed", flywheelSpeed);
        ActiveOpMode.telemetry().addData("kP", kP);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
    }
}

