/*
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
    public static double targetspeed = 2200;
    public static double tolerance = 50;
//    public static double kP = 0.00035, kI = 1E-7, kD = 0, kV = 1.667E-4, kA = 0, kS = 0.00425;
    public static double kP = .00035, kI = 0, kD = 0, kV = 0, kA = 0, kS = 0;
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }
    MotorGroup FlyWheelGroup = new MotorGroup(
            new MotorEx("FlywheelRight"),
            new MotorEx("FlywheelLeft").reversed()
    );


    public final PIDCoefficients pid = new PIDCoefficients(kP,kI,kD);
    public final BasicFeedforwardParameters ff =
            new BasicFeedforwardParameters(kV,kA,kS);

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();

    public void setTargetSpeed(double speed) {
        controller.setGoal(new KineticState(speed, 0));
    public final Command stop = new InstantCommand(() -> controller.setGoal(new KineticState(0,0))).requires(this);

//    public Command superlongshot = new SetPower(FlyWheelGroup,1).requires(this);
//    public Command longshot = new SetPower(FlyWheelGroup,.75).requires(this);
//    public Command shortshot = new SetPower(FlyWheelGroup,.6).requires(this);
//    public Command stop = new SetPower(FlyWheelGroup,0).requires(this);


 */

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

@Configurable

public class FlyWheel implements Subsystem {
    public static final FlyWheel INSTANCE = new FlyWheel();
    public static double tolerance = 50;
    public static double maxvelocity = 2800;
    private FlyWheel() { }
    MotorGroup FlyWheelGroup = new MotorGroup(
            new MotorEx("FlywheelRight"),
            new MotorEx("FlywheelLeft").reversed()
    );

    public static double kP = 1.6667E-4, kI = 0, kD = 0, kV = 0.00038, kA = 0, kS = 0.0425;

    public final PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);
    public final BasicFeedforwardParameters ff =
            new BasicFeedforwardParameters(kV, kA, kS);

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();

    public Command superlongshot = new RunToVelocity(controller, maxvelocity, tolerance).requires(this);
    public Command longshot = new RunToVelocity(controller, 0.7*maxvelocity, tolerance).requires(this);
    public Command shortshot     = new RunToVelocity(controller, 0.5*maxvelocity, tolerance).requires(this);
    public final Command stop = new InstantCommand(() -> controller.setGoal(new KineticState(0,0))).requires(this);


@Override
    public void periodic() {
//        FlyWheelGroup.setPower(controller.calculate(FlyWheelGroup.getState()));
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