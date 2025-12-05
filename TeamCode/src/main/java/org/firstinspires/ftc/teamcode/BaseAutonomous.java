package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousPaths;
import org.firstinspires.ftc.teamcode.subsystems.BaseHook;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;
import java.util.List;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class BaseAutonomous extends NextFTCOpMode {

    // Abstract methods for subclasses to provide specific values
    protected abstract Pose getStartPose();
    protected abstract Pose getScanPose();
    protected abstract Pose getScoring1Pose();
    protected abstract Pose getScoring2Pose();
    protected abstract Pose getPickup1PPGPose();
    protected abstract Pose getPickup2PPGPose();
    protected abstract Pose getPickup1PGPPose();
    protected abstract Pose getPickup2PGPPose();
    protected abstract Pose getPickup1GPPPose();
    protected abstract Pose getPickup2GPPPose();

    protected abstract Command getPpgShot();
    protected abstract Command getPgpShot();
    protected abstract Command getGppShot();
    protected abstract Command getFinalShot();

    public BaseAutonomous() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    // === Timers ===
    private final Timer opmodeTimer = new Timer();

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int SEQUENCE_DETECT_PIPELINE = 0;
    private static final int DISTANCE_DETECT_PIPELINE = 1;
    private static final int DETECTION_TIMEOUT = 1500;

    // === Pathing ===

    private TelemetryManager panelsTelemetry;
    private int foundID   = 0;
    // === Limelight ===
    private Limelight3A limelight;

    // === Path Chains ===
    private PathChain alignScan, scanToScore;
    private PathChain alignShoot, toPickup1PPG, scoopPPG, reversescoopPPG, backToScorePPG, leavePPG;
    private PathChain toPickup1PGP, scoopPGP, reversescoopPGP, backToScorePGP, leavePGP;
    private PathChain toPickup1GPP, scoopGPP, reversescoopGPP, backToScoreGPP, leaveGPP;

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

    // === Path Building ===
    private void buildPaths() {
        AutonomousPaths.PathContainer paths = AutonomousPaths.buildPaths(
                getScanPose(),
                getStartPose(), getScoring1Pose(), getScoring2Pose(),
                getPickup1PPGPose(), getPickup2PPGPose(),
                getPickup1PGPPose(), getPickup2PGPPose(),
                getPickup1GPPPose(), getPickup2GPPPose()
        );
        alignShoot = paths.alignShoot;
        scanToScore = paths.scanToScore;
        alignScan = paths.alignScan;
        toPickup1PPG = paths.toPickup1PPG;
        scoopPPG = paths.scoopPPG;
        reversescoopPPG = paths.reversescoopPPG;
        backToScorePPG = paths.backToScorePPG;
        leavePPG = paths.leavePPG;
        toPickup1PGP = paths.toPickup1PGP;
        scoopPGP = paths.scoopPGP;
        reversescoopPGP = paths.reversescoopPGP;
        backToScorePGP = paths.backToScorePGP;
        leavePGP = paths.leavePGP;
        toPickup1GPP = paths.toPickup1GPP;
        scoopGPP = paths.scoopGPP;
        reversescoopGPP = paths.reversescoopGPP;
        backToScoreGPP = paths.backToScoreGPP;
        leaveGPP = paths.leaveGPP;
    }

    // === AprilTag Detection (runs in start()) ===
    private void detectAprilTag() {
        int timeout = 0;
        while (timeout < DETECTION_TIMEOUT && foundID == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (int i = 0; i < fiducials.size(); i++) {
                        int tagID = fiducials.get(i).getFiducialId();
                        if (tagID == PPG_TAG_ID || tagID == PGP_TAG_ID || tagID == GPP_TAG_ID) {
                            foundID = tagID;
                            log("Detected Tag", tagID);
                            return;
                        }
                    }
                }
            }
            new Delay(5);
            timeout++;
        }
//        if(foundID == 0 && timeout > DETECTION_TIMEOUT)
//            foundID = PGP_TAG_ID;
        log("Warning", "No tag detected – using PGP");
    }

    // ==============================================================
    //  OpMode Lifecycle – ONLY THESE 5 METHODS
    // ==============================================================
    public static void drawOnlyCurrent(Pose robotPose) {
        try {
            Drawing.drawRobot(robotPose);
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setMaxPower(0.8);
        PedroComponent.follower().setStartingPose(getStartPose());
        drawOnlyCurrent(PedroComponent.follower().getPose());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(SEQUENCE_DETECT_PIPELINE);
        limelight.start();

        opmodeTimer.resetTimer();

        buildPaths();

        BaseHook.INSTANCE.BaseHookLock(); // Lock the servo to hold the base plate

        log("Status", "INIT: Ready");
    }

    @Override
    public void onWaitForStart() {
        PedroComponent.follower().setStartingPose(getStartPose());
        new Delay(1);
        log("Starting pose", getStartPose());
        log("Current pose", PedroComponent.follower().getPose());
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
        drawOnlyCurrent(PedroComponent.follower().getPose());
    }

    @Override
    public void onStartButtonPressed() {
            opmodeTimer.resetTimer();
            detectAprilTag(); // This should run before building the command
            PedroComponent.follower().setStartingPose(getStartPose()); // Try setting starting pose again as backup after init.
            drawOnlyCurrent(PedroComponent.follower().getPose());

            //log("Time (s)", String.format("%.2f"));
            log("Status", "START: Running");

            Command runToShoot;
            Command runToScan;
            Command AutonomousPPG;
            Command AutonomousPGP;
            Command AutonomousGPP;
            Command AutonomousRoutine1;

            AutonomousPPG = new SequentialGroup (
                    getPpgShot(),
                    PurpleProtonRobot.INSTANCE.IntakeRun,
                    new FollowPath(toPickup1PPG, true, .9),
                    new FollowPath(scoopPPG, true, 0.3),
                    PurpleProtonRobot.INSTANCE.IntakeStop,
                    new FollowPath(reversescoopPPG, true, 0.9),
                    new FollowPath(backToScorePPG, true, .9),
                    getFinalShot(),
                    new FollowPath(leavePPG, true, .9)
            );
            AutonomousPGP = new SequentialGroup (
                    getPgpShot(),
                    PurpleProtonRobot.INSTANCE.IntakeRun,
                    new FollowPath(toPickup1PGP, true, .9),
                    new FollowPath(scoopPGP, true, 0.3),
                    PurpleProtonRobot.INSTANCE.IntakeStop,
                    new FollowPath(reversescoopPGP, true, 0.9),
                    new FollowPath(backToScorePGP, true, .9),
                    getFinalShot(),
                    new FollowPath(leavePGP, true, .9)
            );
            AutonomousGPP = new SequentialGroup(
                    getGppShot(),
                    PurpleProtonRobot.INSTANCE.IntakeRun,
                    new FollowPath(toPickup1GPP, true, .9),
                    new FollowPath(scoopGPP, true, 0.3),
                    PurpleProtonRobot.INSTANCE.IntakeStop,
                    new FollowPath(reversescoopGPP, true, 0.9),
                    new FollowPath(backToScoreGPP, true, .9),
                    getFinalShot(),
                    new FollowPath(leaveGPP, true, .9)
            );

            runToShoot = new SequentialGroup(
                    new FollowPath(alignShoot, true, .9)
            );
            runToScan = new SequentialGroup(
                    new FollowPath(alignScan, true, .9),
                    new Delay(0.5),
                    new InstantCommand(this::detectAprilTag),
                    new FollowPath(scanToScore, true,.9)
            );
            AutonomousRoutine1 =
                    new SequentialGroup(
                            new SwitchCommand<>(() -> foundID)
                                .withCase(PPG_TAG_ID, runToShoot)
                                .withCase(PGP_TAG_ID, runToShoot)
                                .withCase(GPP_TAG_ID, runToShoot)
                                .withDefault(runToScan),
                        new SwitchCommand<>(() -> foundID)
                                .withCase(PPG_TAG_ID, AutonomousPPG)
                                .withCase(PGP_TAG_ID, AutonomousPGP)
                                .withCase(GPP_TAG_ID, AutonomousGPP)
                                .withDefault(AutonomousGPP)
                );
            PedroComponent.follower().setStartingPose(getStartPose());
            AutonomousRoutine1.schedule();
    }

    @Override
    public void onUpdate() {
            // onUpdate can be simplified. The command scheduler runs automatically.
            // You only need Pedro's update and your telemetry.
            PedroComponent.follower().update();

            if (panelsTelemetry != null) panelsTelemetry.update();

            Pose pose = PedroComponent.follower().getPose();
            drawOnlyCurrent(pose);
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
}
