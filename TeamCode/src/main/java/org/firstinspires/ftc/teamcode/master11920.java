package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="manual11920", group="group")
//@Disabled
public class master11920 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor arm;
    private Servo dumper;
    private Servo marker;
    private DcMotor lift;
    private DcMotor intake;

    private double rotations;

    private double factor = 1;

    private int liftPos;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        arm = hardwareMap.get(DcMotor.class, "arm");
        dumper = hardwareMap.get(Servo.class, "dumper");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        marker = hardwareMap.get(Servo.class, "marker");

        // randomMotor = hardwareMap.get(DcMotor.class, "randomMotor");

        // Encoder Stuff
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        dumper.setDirection(Servo.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        marker.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            liftPos = lift.getCurrentPosition();

            // Drive Train: Control and Speed Change
            setLeftPower((gamepad1.left_stick_y + gamepad2.left_stick_y), factor);
            setRightPower((gamepad1.right_stick_y+ gamepad2.right_stick_y), factor);

            factorChange(.33, 1);

            // Elevator/Lift Control: Up, Down, and else keep at current position
            if (gamepad2.dpad_up && liftPos <= 12000 && gamepad2.left_bumper == false) {
                lift.setPower(1.0);
            }
            else if (gamepad2.dpad_down && liftPos >= -1 && gamepad2.left_bumper == false) {
                lift.setPower(-1.0);
            }
            else if (gamepad2.dpad_up && gamepad2.left_bumper == true) {
                lift.setPower(1.0);
            }
            else if (gamepad2.dpad_down && gamepad2.left_bumper == true) {
                lift.setPower(-1.0);
            }
            else {
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.setPower(0);
            }

            // Lift Reset Encoder
            if (gamepad2.right_bumper) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Dumper Control
            if (gamepad2.x) {
                dumper.setPosition(0.05);
            }
            if (gamepad2.y) {
                dumper.setPosition(.95);
            }

            // Intake Control: Forward, Reverse, and Stop
            if (gamepad1.right_bumper)  {
                intake.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }

            // Arm
            arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // Arm
            if (gamepad1.dpad_right) {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm.setPower(0);
            }
            if (gamepad1.dpad_left) {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setPower(0);
                //int currentPositionArm = arm.getCurrentPosition();
                //arm.setTargetPosition(currentPositionArm);
            }
            telemetry.addData("Lift Encoder Value: ", lift.getCurrentPosition());
            telemetry.addData("Arm Encoder Value: ", arm.getCurrentPosition());
            telemetry.addData("marker position", marker.getPosition());
            telemetry.update();


        }


    }
    public void moveServo(Servo servoName, double position) {
        servoName.setPosition(position);
    }


    public void motorWithEncoder(double targetRotation, DcMotor motor, double power, boolean button) {

        //if A is pressed, step and reset and run the encoder motor at a certain power
        if (button) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setPower(power);
        }

        //always check your position, and turn off the encoder motor if you've reached the desired rotation
        rotations = (motor.getCurrentPosition())/(540);

        if (rotations > targetRotation) {
            motor.setPower(0);

        }

    }

    public void setLeftPower(double leftPower, double parameter) {
        leftFront.setPower(leftPower * parameter);
        leftBack.setPower(leftPower * parameter);
    }
    public void setRightPower(double rightPower, double parameter){
        rightFront.setPower(rightPower * parameter);
        rightBack.setPower(rightPower * parameter);
    }

    public void factorChange(double factor1, double factor2) {
        if (gamepad2.b) {
            factor = factor1;
        }
        if (gamepad2.a) {
            factor = factor2;
        }
    }


}


