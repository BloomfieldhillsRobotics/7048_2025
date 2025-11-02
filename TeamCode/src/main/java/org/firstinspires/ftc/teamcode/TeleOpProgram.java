package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;


@TeleOp(name = "Los Protos")
@Configurable //Panels
public class TeleOpProgram extends NextFTCOpMode {

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    DriverControlledCommand driverControlled = new PedroDriverControlled(
            () -> -Gamepads.gamepad1().leftStickY().get(),
            () -> -Gamepads.gamepad1().leftStickX().get(),
            () -> -Gamepads.gamepad1().rightStickX().get()
    );

    private TelemetryManager telemetryM;
//    private HuskyLensTagDetector tagDetector;
    private final Pose startPose = new Pose(56,8, Math.toRadians(0));
    private final Pose shootPose = new Pose(48, 95, Math.toRadians(180));
    //path to pick up PPG motif
    private final Pose pickUpPPG = new Pose(42, 35, Math.toRadians(180));
    private final Pose pickUpPPGcontrol = new Pose(14.5, 35, Math.toRadians(180));
    //path to pick up PGP motif

    //PPG path chains
    private PathChain PPGfirst;
    private PathChain PPGsecond;
    private void buildPaths() {
        //PGP paths
        PPGfirst = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(startPose, pickUpPPGcontrol, pickUpPPG))
                .setConstantHeadingInterpolation(0.0)
                .build();
        PPGsecond = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickUpPPG, shootPose))
                .setConstantHeadingInterpolation(0.0)
                .build();}
    public Command getPPG() {
        return new SequentialGroup(
                new FollowPath(PPGfirst)
//                , //go to front of artifacts
//                PurpleProtonRobot.INSTANCE.intakeRun, //intake on
//                new FollowPath(PPGsecond), //
//                PurpleProtonRobot.INSTANCE.intakeStop, //intake off
//                PurpleProtonRobot.INSTANCE.shoot
        );
    }
    @Override public void onInit() {
        buildPaths();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override public void onUpdate() {
        // Update Pedro Pathing and Panels every iteration
//        PedroComponent.follower().update();
    }
    public void onStartButtonPressed() {
        driverControlled.schedule();
        PedroComponent.follower().setStartingPose(startPose);
        Gamepads.gamepad1().a()
                .whenBecomesTrue(getPPG());
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