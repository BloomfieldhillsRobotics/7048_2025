
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.conditionals.SwitchCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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
@Disabled
public class AutoProgram extends NextFTCOpMode {
    private TelemetryManager telemetryM;
    public Limelight3A limelight;
    public int at, atag; // April tag code 21, 22 or 23
    
    public AutoProgram() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

//    private HuskyLensTagDetector tagDetector;
    private final Pose startPoseRed1 = new Pose(88,7, Math.toRadians(90));
    private final Pose testPose = new Pose(88,30, Math.toRadians(90));
    private final Pose AprilReadRed1 = new Pose(88, 90, Math.toRadians(90));
    private final Pose ShootRed1 = new Pose(88, 90, Math.toRadians(45));
    private final Pose PickupPPG23r = new Pose(102,84, Math.toRadians(0));
    private final Pose PickupPGP22r = new Pose(102,60, Math.toRadians(0));
    private final Pose PickupGPP21r = new Pose(102,36,Math.toRadians(0));
    private final Pose IntakePPG23ra = new Pose(102,84, Math.toRadians(0));
    private final Pose IntakePPG23rb = new Pose(115,84, Math.toRadians(0));
    private final Pose EndAuto = new Pose(88,62, Math.toRadians(0));

    //PPG path chains
    private PathChain AprilReadRed1Path;
    private PathChain ShootRed1Path;
    private PathChain PickupPPG23rPath;
    private PathChain PickupPGP22rPath;
    private PathChain PickupGPP21rPath;
    private PathChain testPosePath;
    private PathChain testPosePath2;
    private PathChain EndAutoPath;


    private void buildPaths() {
        //PGP paths
        testPosePath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), testPose))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), testPose.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        testPosePath2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), startPoseRed1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), startPoseRed1.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        AprilReadRed1Path = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), AprilReadRed1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), AprilReadRed1.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        ShootRed1Path = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), ShootRed1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), ShootRed1.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        PickupPPG23rPath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), IntakePPG23ra, IntakePPG23rb))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), IntakePPG23ra.getHeading(),IntakePPG23rb.getHeading())
                .setGlobalDeceleration(.1)
                .build();
        EndAutoPath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(PedroComponent.follower().getPose(), EndAuto))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), EndAuto.getHeading())
                .setGlobalDeceleration(.1)
                .build();

    }

    private Command redPPG() {
        return new SequentialGroup(
                PurpleProtonRobot.INSTANCE.IntakeSeq,
                PurpleProtonRobot.INSTANCE.LongShot,
                PurpleProtonRobot.INSTANCE.IntakeSeq,
                PurpleProtonRobot.INSTANCE.LongShot,
                PurpleProtonRobot.INSTANCE.BasketDrop,
                PurpleProtonRobot.INSTANCE.LongShot,
                PurpleProtonRobot.INSTANCE.IntakeRun,
                new FollowPath(PickupPPG23rPath, true, 0.5),
                PurpleProtonRobot.INSTANCE.IntakeStop

        );
    }
    private Command redPGP() {
        return new SequentialGroup(

        );

    }
    private Command redGPP() {
        return new SequentialGroup(

        );
    }


    public Command red1() {
        return new SequentialGroup(
                PurpleProtonRobot.INSTANCE.elevatorDown,
                new FollowPath(AprilReadRed1Path, true, 0.8),
                new Delay(1),
                //atag = at,
                new FollowPath(ShootRed1Path, true, 0.8),

                //new SwitchCommand(() -> "21")
                //    .withCase("21", redGPP())
                  //  .withCase("22", redPGP())
                //    .withCase("23", redPPG())
                //    .withDefault(redGPP()),
                redPPG(),
                new FollowPath(ShootRed1Path, true, 0.8),
                PurpleProtonRobot.INSTANCE.IntakeSeq,
                PurpleProtonRobot.INSTANCE.LongShot,
                PurpleProtonRobot.INSTANCE.IntakeSeq,
                PurpleProtonRobot.INSTANCE.LongShot,
                PurpleProtonRobot.INSTANCE.IntakeSeq,
                PurpleProtonRobot.INSTANCE.LongShot,
                new FollowPath(EndAutoPath)
        );
    }

    public Command testauto() {
          return new SequentialGroup(
                PurpleProtonRobot.INSTANCE.elevatorUp,
                new FollowPath(testPosePath, true, 0.5),
                new Delay(2),
                PurpleProtonRobot.INSTANCE.elevatorDown,
                new FollowPath(testPosePath2, true, 0.5)
                 //new IfElseCommand(() -> at == 21, red1(), red1())
                  //new SwitchCommand(() -> "21")
                    //      .withCase("21", red1())
                      //    .withCase("22", red1())
                        //  .withCase("23", red1())
                          //.withDefault(red1())
        //readAT.update()
        );
    }

    public Command testauto2() {
        return new SequentialGroup(
                PurpleProtonRobot.INSTANCE.elevatorUp,
                new FollowPath(testPosePath, true, 0.5),
                new Delay(2),
                PurpleProtonRobot.INSTANCE.elevatorDown,
                new FollowPath(testPosePath2, true, 0.5)
                //new IfElseCommand(() -> at == 21, red1(), red1())
                //new SwitchCommand(() -> "21")
                //      .withCase("21", red1())
                //    .withCase("22", red1())
                //  .withCase("23", red1())
                //.withDefault(red1())
                //readAT.update()
        );
    }

    public class readAT extends Command {
        public final readAT INSTANCE = new readAT();
        public readAT(){};
        public int at;

        @Override
        public boolean isDone() {
            return false; // whether or not the command is done
        }

        @Override
        public void start() {
            // executed when the command begins
        }

        @Override
        public void update() {
            // executed on every update of the command
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
                } //return at;
            } else {
                telemetry.addData("Limelight", "No data available");
                //return 21;
            }
        }

        @Override
        public void stop(boolean interrupted) {
            // executed when the command ends
        }

    }

    @Override public void onInit() {
        PedroComponent.follower().setStartingPose(startPoseRed1);
        PedroComponent.follower().setMaxPower(0.8);
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
    @Override
    public void onStartButtonPressed() {
        //red1().schedule();
        new SequentialGroup(
            testauto(),
            testauto2()
        ).schedule();
        /*
        switch (readAT()) {
            case 21:
                telemetry.addData("at", readAT());
                telemetry.update();
                //testauto().schedule();
                break;
            case 22:
                //testauto().schedule();
                break;
            case 23:
                //testauto().schedule();
                break;
        }

         */
    }

}