package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Los Protos")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private final MotorEx frontLeftMotor = new MotorEx("front left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back left");
    private final MotorEx backRightMotor = new MotorEx("back right").reversed();
    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();
        Gamepads.gamepad1().a()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.intakeRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.intakeStop);
        Gamepads.gamepad1().x()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.score);
        Gamepads.gamepad1().y()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorUp);
        Gamepads.gamepad1().b()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.FlyWheelRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
    }
}