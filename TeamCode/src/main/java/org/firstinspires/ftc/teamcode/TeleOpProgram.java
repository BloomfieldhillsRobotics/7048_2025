
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;

import java.util.List;


@TeleOp(name = "Los Protos")
@Configurable //Panels
public class TeleOpProgram extends NextFTCOpMode {
    private double dynamicFlywheelSpeed = 0.0; // Variable to hold our calculated speed
    private TelemetryManager telemetryM;
    private Limelight3A limelight;
    private double calculateSpeedFromVerticalOffset(double ty) {
        // THIS IS A PLACEHOLDER FORMULA! TUNE THIS FOR YOUR ROBOT!
        // For example, a simple linear mapping:
        // Let's say at ty = -15 degrees (close), you need 1800 RPM.
        // And at ty = 5 degrees (far), you need 3000 RPM.
        // You can use a linear equation: y = mx + b
        // m = (3000-1800) / (5 - (-15)) = 1200 / 20 = 60
        // b = 1800 - (60 * -15) = 1800 + 900 = 2700
        double speed = (60 * ty) + 2700;

        // Clamp the speed to a safe range
        return Math.max(1500, Math.min(3200, speed));
    }
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


    @Override public void onInit() {
        driverControlled.schedule();
        //PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().setMaxPower(1);
        PedroComponent.follower().update();

        //buildPaths();
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
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());


        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            dynamicFlywheelSpeed = calculateSpeedFromVerticalOffset(result.getTy());
            //telemetry.addData("LL Latency", captureLatency + targetingLatency);
           // telemetry.addData("Parse Latency", parseLatency);
            //telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());
/*
            // Access barcode results
            List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult br : barcodeResults) {
                telemetry.addData("Barcode", "Data: %s", br.getData());
            }

            // Access classifier results
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
            }

            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
            }

             // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
*/
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }


        } else {
            telemetry.addData("Limelight", "No data available");
            telemetry.addData("Dynamic Flywheel Target", "%.2f", dynamicFlywheelSpeed);
        }

        Pose pose = PedroComponent.follower().getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeading());
        telemetryM.debug("position", pose.getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        Drawing.drawOnlyCurrent(pose);
        telemetryM.update();
        telemetry.update();

    }
    public void onStartButtonPressed() {
        PedroComponent.follower().startTeleopDrive();
       // Gamepads.gamepad1().a()
      //          .whenBecomesTrue(getGPP());
        Gamepads.gamepad2().b()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.IntakeRun)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.IntakeStop);
        //Gamepads.gamepad2().a()
        //        .whenBecomesTrue(PurpleProtonRobot.INSTANCE.runIntakeBackwards)
         //       .whenBecomesFalse(PurpleProtonRobot.INSTANCE.intakeStop);
        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorUp)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.elevatorDown);
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.ShortShot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.LongShot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.superlongshot)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        //Gamepads.gamepad2().b()
          //      .whenBecomesTrue(PurpleProtonRobot.INSTANCE.shortshot)
            //    .whenBecomesFalse(PurpleProtonRobot.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().y()
                .whenBecomesTrue(PurpleProtonRobot.INSTANCE.BasketDrop)
                .whenBecomesFalse(PurpleProtonRobot.INSTANCE.BasketUp);
    }
}