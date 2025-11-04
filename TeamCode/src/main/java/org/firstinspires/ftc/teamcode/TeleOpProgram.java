
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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
    private TelemetryManager telemetryM;
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    DriverControlledCommand driverControlled = new PedroDriverControlled(
           // () -> -Gamepads.gamepad1().leftStickY().get(),
          //  () -> -Gamepads.gamepad1().leftStickX().get(),
          //  () -> -Gamepads.gamepad1().rightStickX().get()
             Gamepads.gamepad1().leftStickY().negate(), // changed to this as above was giving runtime exceptions
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate()
    );

//    private HuskyLensTagDetector tagDetector;
    private final Pose startPose = new Pose(56,8, Math.toRadians(90));
    private final Pose shootPose = new Pose(48, 95, Math.toRadians(135));
    //path to setup GPP pickup
    private final Pose PickupGPPFirst = new Pose(48,35, Math.toRadians(180));
    //Pickup GPP
    private final Pose PickupGPPSecond = new Pose(20,36, Math.toRadians(180));
    //Avoid Obstacles
    private final Pose PickupGppAvoidObstacles = new Pose(69,47,Math.toRadians(180));
    //Return to Base
    private final Pose ReturntoBase = new Pose(39,34, Math.toRadians(180));

    //PPG path chains
    private PathChain PickupGPPFirstPath;
    private PathChain PickupGPPSecondPath;
    private PathChain PickupGPPShoot;
    private PathChain GPPReturnToBase;
    private PathChain PathPickupGppAvoidObstacles;

    private void buildPaths() {
        //PGP paths
        PickupGPPFirstPath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), PickupGPPFirst))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), PickupGPPFirst.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        PickupGPPSecondPath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PickupGPPFirst, PickupGPPSecond))
                .setGlobalDeceleration(.025)
                .build();
        PathPickupGppAvoidObstacles = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), PickupGppAvoidObstacles))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), PickupGppAvoidObstacles.getHeading())
                .setGlobalDeceleration(.5)
                .build();
        PickupGPPShoot = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), shootPose))
                .setGlobalDeceleration(.5)
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), shootPose.getHeading())
                .build();
        GPPReturnToBase = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), ReturntoBase))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), ReturntoBase.getHeading())
                .setGlobalDeceleration(.5)
                .build();
    }

    public Command getGPP() {
        return new SequentialGroup(
                PurpleProtonRobot.INSTANCE.elevatorDown,
                new FollowPath(PickupGPPFirstPath),
                new Delay(1),
                PurpleProtonRobot.INSTANCE.intakeRun,
                new Delay(1),
                new FollowPath(PickupGPPSecondPath),
                new Delay(1),
                PurpleProtonRobot.INSTANCE.intakeStop,
                new Delay(1),
                new FollowPath(PathPickupGppAvoidObstacles),
                new Delay(1),
                new FollowPath(PickupGPPShoot),
                new Delay(1),
                PurpleProtonRobot.INSTANCE.shoot,
                PurpleProtonRobot.INSTANCE.shoot,
                PurpleProtonRobot.INSTANCE.shoot,
                PurpleProtonRobot.INSTANCE.intakeStop,
                new FollowPath(GPPReturnToBase)
        );
    }
    @Override public void onInit() {
        driverControlled.schedule();
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().update();
        buildPaths();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override public void onUpdate() {
        // Update Pedro Pathing and Panels every iteration
        PedroComponent.follower().update();
        BindingManager.update();
        // These loop the movements of the robot, these must be called continuously in order to work
        // Feedback to Driver Hub for debugging
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("y", PedroComponent.follower().getPose().getY());
        telemetry.addData("heading", PedroComponent.follower().getPose().getHeading());
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.update();
        telemetry.update();

    }
    public void onStartButtonPressed() {
        PedroComponent.follower().startTeleopDrive();
        Gamepads.gamepad1().a()
                .whenBecomesTrue(getGPP());
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