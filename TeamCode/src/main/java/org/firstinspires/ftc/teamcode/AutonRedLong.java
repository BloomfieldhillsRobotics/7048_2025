
package org.firstinspires.ftc.teamcode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RedAutonLong")
public class AutonRedLong extends NextFTCOpMode {
    public AutonRedLong() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    // === Timers ===
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final Timer pathTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    // === Poses ===
    private final Pose startPose = new Pose(88, 7, Math.toRadians(90));

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 0;
    private static final int DETECTION_TIMEOUT = 100;

    // === Pathing ===

    private TelemetryManager panelsTelemetry;
    private int pathState = 0;
    private int foundID   = 0;

    // === Limelight ===
    private Limelight3A limelight;

    // === Path Chains ===
    private PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;

    // === Shooting ===
    private boolean isShooting = false;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // === Timer Class ===
    public static class Timer {
        private final ElapsedTime t = new ElapsedTime();
        public void resetTimer() { t.reset(); }
        public double getElapsedTimeSeconds() { return t.seconds(); }
    }

    // === Logging ===
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsTelemetry != null) panelsTelemetry.debug(caption + ": " + value);
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        log("Path State", pathState);
    }

    // === Path Building ===
    private void buildPaths() {
        Pose scoring1 = new Pose(90, 90, Math.toRadians(45));
        Pose scoring2 = new Pose(96, 49, Math.toRadians(45));

        // PPG
        Pose pickup1PPG = new Pose(100, 83, Math.toRadians(0));
        Pose pickup2PPG = new Pose(124, 83, Math.toRadians(0));

        alignPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PPG))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PPG.getHeading())
                .build();

        scoopPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PPG, pickup2PPG))
                .setConstantHeadingInterpolation(pickup1PPG.getHeading())
                .build();

        backToScorePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PPG, scoring1))
                .setLinearHeadingInterpolation(pickup2PPG.getHeading(), scoring1.getHeading())
                .build();

        leavePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();

        // PGP
        Pose pickup1PGP = new Pose(100, 59, Math.toRadians(0));
        Pose pickup2PGP = new Pose(124, 59, Math.toRadians(0));

        alignPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PGP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PGP.getHeading())
                .build();

        scoopPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PGP, pickup2PGP))
                .setConstantHeadingInterpolation(pickup1PGP.getHeading())
                .build();

        backToScorePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PGP, scoring1))
                .setLinearHeadingInterpolation(pickup2PGP.getHeading(), scoring1.getHeading())
                .build();

        leavePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();



        alignGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();
        // GPP
        Pose pickup1GPP = new Pose(100, 39, Math.toRadians(0));
        Pose pickup2GPP = new Pose(124, 39, Math.toRadians(0));

        toPickup1GPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PGP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1GPP.getHeading())
                .build();

        scoopGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1GPP, pickup2GPP))
                .setConstantHeadingInterpolation(pickup1GPP.getHeading())
                .build();

        backToScoreGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PGP, scoring1))
                .setLinearHeadingInterpolation(pickup2GPP.getHeading(), scoring1.getHeading())
                .build();

        leaveGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();
    }

    // === AprilTag Detection (runs in start()) ===
    private void detectAprilTag() {
        int timeout = 0;
        while (timeout < DETECTION_TIMEOUT && foundID == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagID = fiducials.get(0).getFiducialId();
                    if (tagID == PPG_TAG_ID || tagID == PGP_TAG_ID || tagID == GPP_TAG_ID) {
                        foundID = tagID;
                        log("Detected Tag", tagID);
                        return;
                    }
                }
            }
            new Delay(5);
            timeout++;
        }
        if(foundID == 0 && timeout > DETECTION_TIMEOUT)
            foundID = PGP_TAG_ID;
        log("Warning", "No tag detected – using PPG");
    }

    // ==============================================================
    //  OpMode Lifecycle – ONLY THESE 5 METHODS
    // ==============================================================

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setMaxPower(0.75);
        PedroComponent.follower().setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        opmodeTimer.resetTimer();

        buildPaths();

        log("Status", "INIT: Ready");
    }

    @Override
    public void onWaitForStart() {
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
            opmodeTimer.resetTimer();
            detectAprilTag(); // This should run before building the command
            log("Status", "START: Running");

            // 1. Create a placeholder for the command we are about to build
            Command autonomousCommand;

            // 2. Build the correct sequence of commands based on the detected tag
            switch (foundID) {
                case PPG_TAG_ID:
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignPPG, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.BasketDrop,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            new FollowPath(alignPPG, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeRun,
                            new FollowPath(toPickup1PPG, true, .8),
                            new FollowPath(scoopPPG, true, 0.3),
                            PurpleProtonRobot.INSTANCE.IntakeStop,
                            new FollowPath(backToScorePPG, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            new FollowPath(leavePPG, true, .8)
                    );
                    break;
                case PGP_TAG_ID:
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignPGP, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.BasketDrop,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            new FollowPath(alignPGP, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeRun,
                            new FollowPath(toPickup1PGP, true, .8),
                            new FollowPath(scoopPGP, true, 0.3),
                            PurpleProtonRobot.INSTANCE.IntakeStop,
                            new FollowPath(backToScorePGP, true, .8),
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            new FollowPath(leavePGP, true, .8)
                    );
                    break;
                case GPP_TAG_ID:
                default: // Default to GPP if something goes wrong
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignGPP, true, .8),
                            PurpleProtonRobot.INSTANCE.BasketDrop,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeRun,
                            new FollowPath(toPickup1GPP, true, .8),
                            new FollowPath(scoopGPP, true, 0.3),
                            PurpleProtonRobot.INSTANCE.IntakeStop,
                            new FollowPath(backToScoreGPP, true, .8),
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            PurpleProtonRobot.INSTANCE.IntakeSeq,
                            PurpleProtonRobot.INSTANCE.LongShot,
                            new FollowPath(leaveGPP, true, .8)
                    );
                    break;
            }

            // 3. Schedule the one, big command to run. The scheduler handles the rest.
            autonomousCommand.schedule();
    };

    @Override
    public void onUpdate() {
            // onUpdate can be simplified. The command scheduler runs automatically.
            // You only need Pedro's update and your telemetry.
            PedroComponent.follower().update();

            if (panelsTelemetry != null) panelsTelemetry.update();

            Pose pose = PedroComponent.follower().getPose();
            double normH = Math.toDegrees((pose.getHeading() + 2 * Math.PI) % (2 * Math.PI));

            log("X",            String.format("%.2f", pose.getX()));
            log("Y",            String.format("%.2f", pose.getY()));
            log("Heading",      String.format("%.2f°", normH));
            log("Found ID",     foundID);
            log("Time (s)",     String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));

            telemetry.update();
    }

    @Override
    public void onStop() {
        if (limelight != null) limelight.stop();
        log("Status", "STOP: Done");
    }
};
