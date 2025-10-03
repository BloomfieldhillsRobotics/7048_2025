package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "FlywheelMotors", group = "Tutorials")
public class FlywheelMotors extends LinearOpMode {


    private static final double LIFT_UP_POS = 0.1; // Example up position
    private static final double LIFT_DOWN_POS = 0.9; // Example down position
    // Declare a variable for our motor.
    private DcMotor flywheelLeft = null;
    private DcMotor flywheelRight = null;
    private DcMotor intakeMotor = null;
    private Servo liftServo = null;


    @Override
    public void runOpMode() {
        // Get a reference to the motor defined in the hardware map.
        // The name "test_motor" must match the name in your configuration file.
        flywheelLeft = hardwareMap.get(DcMotor.class, "FlywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "FlywheelRight");
        liftServo = hardwareMap.get(Servo.class, "LiftServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");




        // Set the motor direction. You may need to reverse this depending on your setup.
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        // Send a message to the Driver Station indicating the robot is ready.
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the driver to press PLAY.
        waitForStart();


        // The OpMode is active until the driver presses STOP.
        while (opModeIsActive()) {
            // Read the left joystick's Y-axis value from gamepad 1.
            double motorPower = gamepad1.left_stick_y;
            double motorintakePower = gamepad1.right_stick_y;


            // Send the joystick value to the motor's power.
            // Power values range from -1.0 to +1.0.
            flywheelLeft.setPower(motorPower);
            flywheelRight.setPower(motorPower);
            intakeMotor.setPower(motorintakePower);

            if (gamepad1.x) {
                liftServo.setPosition(LIFT_DOWN_POS); // Move lift down
                intakeMotor.setPower(0.7); //Have intake go
                sleep(2000); // Wait for 2 seconds
                liftServo.setPosition(LIFT_UP_POS); // Move lift up
                intakeMotor.setPower(0); //stop the intake motor
                flywheelLeft.setPower(-1); //start up the flywheel motors
                flywheelRight.setPower(-1); //start up the flywheel motors
                sleep(2000); // Wait for 2 seconds
                flywheelLeft.setPower(0); //stop the flywheel motors
                flywheelRight.setPower(0); //stop the flywheel motors
                liftServo.setPosition(LIFT_DOWN_POS); // Move lift down
            }


            // Example: Control lift with gamepad 'a' and 'b' buttons
            if (gamepad1.a) {
                liftServo.setPosition(LIFT_UP_POS); // Move lift up
                telemetry.addData("Lift", "Up");
            } else if (gamepad1.b) {
                liftServo.setPosition(LIFT_DOWN_POS); // Move lift down
                telemetry.addData("Lift", "Down");
            }




            // Display the current motor power on the Driver Station.
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }


        // When the OpMode is stopped, set the motor power to 0.
        flywheelRight.setPower(0);
        flywheelRight.setPower(0);
    }
}



