package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable

public class FlyWheel implements Subsystem {
    public double targetspeed = 2400;
    public static double deadband = 5;
    public static double kP = 0.0002, kI = 0, kD = 0, kV = 0.0004, kA = 0, kS = 0;
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }
    MotorGroup FlyWheelGroup = new MotorGroup(
            new MotorEx("FlywheelRight")
            //new MotorEx("FlywheelLeft").reversed()
    );

    public final PIDCoefficients pid = new PIDCoefficients(kP,kI,kD);
    public final BasicFeedforwardParameters ff =
            new BasicFeedforwardParameters(kV,kA,kS);

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();
    public void setTargetSpeed(double speed) {
        targetspeed=speed;
        //controller.setGoal(new KineticState(speed, 0));
        //new InstantCommand(() -> controller.setGoal(new KineticState(0,targetspeed))).requires(this);
        new RunToVelocity(controller, targetspeed, deadband).requires(this);
    }
//
    public Command superlongshot = new RunToVelocity(controller, targetspeed, deadband).requires(this);
    public Command longshot = new RunToVelocity(controller, 2000, deadband).requires(this);
    public Command shortshot     = new RunToVelocity(controller, 1500, deadband).requires(this);
    public final Command stop = new InstantCommand(() -> controller.setGoal(new KineticState(0,0))).requires(this);

    @Override
    public void periodic() {
        FlyWheelGroup.setPower(controller.calculate(FlyWheelGroup.getState()));
        ActiveOpMode.telemetry().addData("Targetspeed", targetspeed);
        ActiveOpMode.telemetry().addData("Flywheel State", FlyWheelGroup.getState());
        ActiveOpMode.telemetry().addData("Flywheel Speed Target", controller.getGoal());
        ActiveOpMode.telemetry().addData("FlyWheel Calculate", controller.calculate(FlyWheelGroup.getState()));
        ActiveOpMode.telemetry().addData("kI", kI);
        ActiveOpMode.telemetry().addData("kI", kI);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
        ActiveOpMode.telemetry().addData("kP", kP);
    }
}