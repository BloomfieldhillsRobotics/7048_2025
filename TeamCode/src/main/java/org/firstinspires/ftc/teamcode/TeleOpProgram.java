package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;


@TeleOp(name = "Los Protos")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

//    private HuskyLensTagDetector tagDetector;

    @Override
    public void onInit() {
    }
    public void onUpdate() {
    }
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();
        Gamepads.gamepad2().b()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.intakeRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.intakeStop);
        Gamepads.gamepad2().a()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.runIntakeBackwards)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.runIntakeBackwards);
        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorUp)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.ShortShot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.longshot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.superlongshot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
    }
}