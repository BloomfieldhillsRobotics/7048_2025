
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

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RedAutonWPedro")
public class AutonRed extends NextFTCOpMode {
    public AutonRed() {
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
    private final Pose startPose = new Pose(85, 9, Math.toRadians(270));

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 5;
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

    // === Path State Machine ===
    private void autonomousPathUpdate() {
        if (foundID == 0) return;

        switch (pathState) {
            case 0: /* This case starts the alignment to the scoring position */
                if (true) {
                    switch (foundID) {
                        case PPG_TAG_ID: PedroComponent.follower().followPath(alignPPG, true); break;
                        case PGP_TAG_ID: PedroComponent.follower().followPath(alignPGP, true); break;
                        case GPP_TAG_ID: PedroComponent.follower().followPath(alignGPP, true); break;
                    }
                    setPathState(1);
                }
                break;

            case 1: /* This case waits for return to scoring and starts shooting */
                if (!PedroComponent.follower().isBusy()) {
                    setPathState(2);
                }
                break;

            case 2: /* This case waits for alignment to finish and moves to pickup 1 */
                if (!isShooting) {
                    switch (foundID) {
                        case PPG_TAG_ID: PedroComponent.follower().followPath(toPickup1PPG, true); break;
                        case PGP_TAG_ID: PedroComponent.follower().followPath(toPickup1PGP, true); break;
                        case GPP_TAG_ID: PedroComponent.follower().followPath(toPickup1GPP, true); break;
                    }
                    setPathState(3);
                }
                break;

            case 3: /* This case waits for robot to reach pickup 1 and scoops the sample */
                if (!PedroComponent.follower().isBusy()) {
                    switch (foundID) {
                        case PPG_TAG_ID: PedroComponent.follower().followPath(scoopPPG, true); break;
                        case PGP_TAG_ID: PedroComponent.follower().followPath(scoopPGP, true); break;
                        case GPP_TAG_ID: PedroComponent.follower().followPath(scoopGPP, true); break;
                    }
                    setPathState(4);
                }
                break;

            case 4: /* This case waits for scoop to finish and returns to scoring position */

                if (!PedroComponent.follower().isBusy()) {
                    PedroComponent.follower().setMaxPower(0.75);
                    switch (foundID) {
                        case PPG_TAG_ID: PedroComponent.follower().followPath(backToScorePPG, true); break;
                        case PGP_TAG_ID: PedroComponent.follower().followPath(backToScorePGP, true); break;
                        case GPP_TAG_ID: PedroComponent.follower().followPath(backToScoreGPP, true); break;
                    }
                    setPathState(5);
                }
                break;

            case 5: /* This case waits for return to scoring and starts shooting */
                if (!PedroComponent.follower().isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: /* This case waits for shooting to complete */
                if (!isShooting) {
                    PedroComponent.follower().setMaxPower(1);
                    switch (foundID) {
                        case PPG_TAG_ID: PedroComponent.follower().followPath(leavePPG, true); break;
                        case PGP_TAG_ID: PedroComponent.follower().followPath(leavePGP, true); break;
                        case GPP_TAG_ID: PedroComponent.follower().followPath(leaveGPP, true); break;
                    }
                    setPathState(7);
                }
                break;

            case 7: /* This case waits for leave path to finish */
                if (!PedroComponent.follower().isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        log("Path State", pathState);
    }

    // === Path Building ===
    private void buildPaths() {
        Pose scoring1 = new Pose(90, 90, Math.toRadians(220));
        Pose scoring2 = new Pose(96, 49, Math.toRadians(220));

        // PPG
        Pose pickup1GPP = new Pose(100, 39, Math.toRadians(0));
        Pose pickup2GPP = new Pose(124, 39, Math.toRadians(0));

        alignPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1GPP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1GPP.getHeading())
                .build();

        scoopPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1GPP, pickup2GPP))
                .setConstantHeadingInterpolation(pickup1GPP.getHeading())
                .build();

        backToScorePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2GPP, scoring1))
                .setLinearHeadingInterpolation(pickup2GPP.getHeading(), scoring1.getHeading())
                .build();

        leavePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();

        // PGP
        Pose pickup1PPG = new Pose(100, 83, Math.toRadians(0));
        Pose pickup2PPG = new Pose(124, 83, Math.toRadians(0));

        alignPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PPG))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PPG.getHeading())
                .build();

        scoopPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PPG, pickup2PPG))
                .setConstantHeadingInterpolation(pickup1PPG.getHeading())
                .build();

        backToScorePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PPG, scoring1))
                .setLinearHeadingInterpolation(pickup2PPG.getHeading(), scoring1.getHeading())
                .build();

        leavePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();

        // GPP
        Pose pickup1PGP = new Pose(100, 59, Math.toRadians(0));
        Pose pickup2PGP = new Pose(124, 59, Math.toRadians(0));

        alignGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1GPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PGP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PGP.getHeading())
                .build();

        scoopGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PGP, pickup2PGP))
                .setConstantHeadingInterpolation(pickup1PGP.getHeading())
                .build();

        backToScoreGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PGP, scoring1))
                .setLinearHeadingInterpolation(pickup2PGP.getHeading(), scoring1.getHeading())
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
        foundID = PPG_TAG_ID;
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
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        new Delay(5);
        detectAprilTag();
        setPathState(0);
        log("Status", "START: Running");
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        autonomousPathUpdate();

        if (panelsTelemetry != null) panelsTelemetry.update();

        Pose pose = PedroComponent.follower().getPose();
        double normH = Math.toDegrees((pose.getHeading() + 2 * Math.PI) % (2 * Math.PI));

        log("X",            String.format("%.2f", pose.getX()));
        log("Y",            String.format("%.2f", pose.getY()));
        log("Heading",      String.format("%.2f°", normH));
        log("Path State",   pathState);
        log("Found ID",     foundID);
        log("Time (s)",     String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));

        telemetry.update();
    }

    @Override
    public void onStop() {
        if (limelight != null) limelight.stop();
        log("Status", "STOP: Done");
    }
}
