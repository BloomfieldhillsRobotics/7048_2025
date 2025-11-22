package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
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
    public static double deadband = 20;
    public static double kP = 0.001, kI = 0, kD = 0, kV = 0.00045, kA = 0, kS = 0;
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }

    private MotorEx FlyWheelRight = new MotorEx("FlywheelRight");
    private MotorEx FlyWheelLeft = new MotorEx("FlywheelLeft");

   //MotorGroup FlyWheelGroup = new MotorGroup(
            //new MotorEx("FlywheelRight"),
            //new MotorEx("FlywheelLeft").reversed()
    //        FlyWheelRight,
    //      FlyWheelLeft.reversed()
    //);

    public final PIDCoefficients pid = new PIDCoefficients(kP,kI,kD);
    public final BasicFeedforwardParameters ff =
            new BasicFeedforwardParameters(kV,kA,kS);

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();

    private final ControlSystem controllerright = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();

    private final ControlSystem controllerleft = ControlSystem.builder()
            .velPid(pid)
            .basicFF(ff)
            .build();

    public void setTargetSpeed(double speed) {
        targetspeed = speed;
        // This is the correct way to set the target. The periodic() method
        // will handle the rest.
        // The previous way was passing deadband as an acceleration, not a deadband
        //new RunToVelocity(controller, targetspeed, deadband).requires(this);
        controllerright.setGoal(new KineticState(0,targetspeed,0));
        controllerleft.setGoal(new KineticState(0,-targetspeed,0));
    }

    //new RunToVelocity(controller, targetspeed, deadband).requires(this);
    //public Command superlongshot = new SequentialGroup(
    //        new RunToVelocity(controllerright, 2300, deadband).requires(this),
     //       new RunToVelocity(controllerleft, -2300, deadband).requires(this)
    //);
    public Command superlongshot = new InstantCommand(() -> {
        setTargetSpeed(2300);
    });
    public Command longshot = new InstantCommand(() -> {
        setTargetSpeed(1800);
    });
    public Command shortshot = new InstantCommand(() -> {
        setTargetSpeed(1200);
    });
    public Command backwards = new InstantCommand(() -> {
        setTargetSpeed(-300);
    });
    public Command stop = new InstantCommand(() -> {
        setTargetSpeed(0);
    });
    //public Command shortshot     = new RunToVelocity(controller, 1500, deadband).requires(this);
    //public Command stop = new SequentialGroup(
      //      new InstantCommand(() -> controllerright.setGoal(new KineticState(0,0))).requires(this),
        //    new InstantCommand(() -> controllerleft.setGoal(new KineticState(0,0))).requires(this)
    //);
    //public final Command stop = new InstantCommand(() -> controller.setGoal(new KineticState(0,0))).requires(this);

    @Override
    public void periodic() {
        FlyWheelLeft.setPower(controllerleft.calculate(FlyWheelLeft.getState()));
        FlyWheelRight.setPower(controllerright.calculate(FlyWheelRight.getState()));
        ActiveOpMode.telemetry().addData("Targetspeed", targetspeed);
       // ActiveOpMode.telemetry().addData("Flywheel State", FlyWheelGroup.getState());
        //ActiveOpMode.telemetry().addData("Flywheel Speed Target", controller.getGoal());
       // ActiveOpMode.telemetry().addData("FlyWheel Calculate", controller.calculate(FlyWheelGroup.getState()));
        ActiveOpMode.telemetry().addData("Flywheel Right velocity", FlyWheelRight.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel Left velocity", FlyWheelLeft.getVelocity());
        ActiveOpMode.telemetry().addData("Flywheel Right calculate", controllerright.calculate(FlyWheelRight.getState()));
        ActiveOpMode.telemetry().addData("Flywheel Left velocity", controllerleft.calculate(FlyWheelLeft.getState()));
        ActiveOpMode.telemetry().addData("kI", kI);
        ActiveOpMode.telemetry().addData("kI", kI);
        ActiveOpMode.telemetry().addData("kD", kD);
        ActiveOpMode.telemetry().addData("kV", kV);
        ActiveOpMode.telemetry().addData("kA", kA);
        ActiveOpMode.telemetry().addData("kS", kS);
        ActiveOpMode.telemetry().addData("kP", kP);
    }
}