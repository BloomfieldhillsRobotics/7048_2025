
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;

import java.util.List;

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

@Autonomous(name = "Autonomous 1", preselectTeleOp = "Los Protos")
@Configurable //Panels
public class AutoProgram extends NextFTCOpMode {
    private TelemetryManager telemetryM;
    private Limelight3A limelight;
    public int at; // April tag code 21, 22 or 23
    
    public AutoProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

//    private HuskyLensTagDetector tagDetector;
    private final Pose startPose = new Pose(88,7, Math.toRadians(90));
    private final Pose aprilRead = new Pose(88, 86, Math.toRadians(90));
    private final Pose shootPose = new Pose(88, 86, Math.toRadians(45));
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
    private PathChain startPosePath;
    private PathChain shootPosePath;
    private PathChain aprilReadPath;

    private void buildPaths() {
        //PGP paths
        startPosePath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), startPose))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), startPose.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        shootPosePath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), shootPose))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), shootPose.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        aprilReadPath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), aprilRead))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), aprilRead.getHeading())
                .setGlobalDeceleration(.1)
                .build();
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
    public void getGPP() {
        new SequentialGroup(
                PurpleProtonRobot.INSTANCE.elevatorDown,
                new FollowPath(aprilReadPath),
                new FollowPath(shootPosePath),
                PurpleProtonRobot.INSTANCE.shoot,
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
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().update();
        buildPaths();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    @Override public void onUpdate() {
        // Update Pedro Pathing and Panels every iteration
        PedroComponent.follower().update();
        BindingManager.update();

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            //telemetry.addData("LL Latency", captureLatency + targetingLatency);
            // telemetry.addData("Parse Latency", parseLatency);
            //telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if ((fr.getFiducialId() > 20) && (fr.getFiducialId() < 24)) {
                    at = fr.getFiducialId();
                }
                else { at = 21; }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        getGPP();

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
    @Override public void onStartButtonPressed() {

    }
}