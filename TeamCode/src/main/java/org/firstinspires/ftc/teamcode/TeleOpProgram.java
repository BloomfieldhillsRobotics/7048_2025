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

import org.firstinspires.ftc.teamcode.subsystems.HuskyLensTagDetector;
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
    private final MotorEx frontLeftMotor = new MotorEx("front left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back left");
    private final MotorEx backRightMotor = new MotorEx("back right").reversed();

    private HuskyLensTagDetector tagDetector;

    @Override
    public void onInit() {
        tagDetector = new HuskyLensTagDetector(hardwareMap, "huskylense");
        telemetry.addData("camear Status", "Initialized Point Huskylens at tag");
    }
    public void onUpdate() {
        tagDetector.scanForTags();
        // Check if a tag was found and report its info
        if (tagDetector.tagWasDetected()) {
            telemetry.addData("Tag Detected!", "");
            telemetry.addData("ID", tagDetector.getDetectedTagId());
            telemetry.addData("Position", "X: %d, Y: %d", tagDetector.getDetectedTagX(), tagDetector.getDetectedTagY());
            telemetry.addData("Size", "Width: %d, Height: %d", tagDetector.getDetectedTagWidth(), tagDetector.getDetectedTagHeight());
        } else {
            telemetry.addData("No Tag Detected", "");
        }
        telemetry.update();
    }
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                () -> -Gamepads.gamepad1().leftStickX().get(),  //todo fix so we don't use negative
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();
        Gamepads.gamepad2().a()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.intakeRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.intakeStop);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.score);
        Gamepads.gamepad2().y()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorUp);
        Gamepads.gamepad2().b()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.FlyWheelRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
    }
}