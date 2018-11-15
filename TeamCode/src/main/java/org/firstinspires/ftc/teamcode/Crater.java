package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CraterRiskyNoDrop", group = "11920")

public class Crater extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo marker = null;
    double distance;
    double LFdistance;
    double LBdistance;
    double RFdistance;
    double RBdistance;
    double diameter = 6;
    boolean stop = false;

    public void runOpMode(){
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        arm = hardwareMap.get(DcMotor.class,"arm");
        marker = hardwareMap.get(Servo.class, "marker");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Needs to lower
            moveDist(arm,-0.5,1200);

            forwardDist(0.4, 6);

            sleep(100);

            turnDeg(-0.4,0.25);

            sleep(100);

            forwardDist(0.4, 30);

            sleep(100);

            //forwardDist(-0.4, 8);

            turnDeg(0.4,0.6);

            sleep(100);

            forwardDist(-0.5, 72);

            sleep(100);

            turnDeg(0.4, 1.0);

            marker.setPosition(0.65);

            sleep(1000);

            marker.setPosition(0);

            //forwardDist(-0.4, 6);

            // Tuned until here

            sleep(100);

            turnDeg(0.4,1.6);

            moveDist(arm,0.5,600);

            forwardDist(0.5,100);



            sleep(30000);


        }


    }

    // FUNCTIONS:

    public void resetEncoder(DcMotor motorname){
        motorname.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorname.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDrivetrain(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void forward(double power){
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    public void turn(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power * -1);
        rightBack.setPower(power * -1);
    }


    /*
    0.25 rotations ~ 40-45 deg
    0.5 rotations ~ 85 deg
    0.75 rotations ~ 120-125 deg
    1 rotation ~ 135-140 deg
    Not recommended to make rotations any larger. It will become inaccurate. Ok got it
    */

    public void turnDeg(double power, double rot){
        resetDrivetrain();


        stop = false;

        if (!stop) {
            turn(power);

        }
        while (!stop){
            LFdistance = Math.abs(leftFront.getCurrentPosition()/1680.0);
            LBdistance = Math.abs(leftBack.getCurrentPosition()/1680.0);
            RFdistance = Math.abs(rightFront.getCurrentPosition()/1680.0);
            RBdistance = Math.abs(rightBack.getCurrentPosition()/1680.0);
            if (LFdistance >= rot){
                leftFront.setPower(0);
            }
            if (LBdistance >= rot){
                leftBack.setPower(0);
            }
            if (RFdistance >= rot){
                rightFront.setPower(0);
            }
            if (RBdistance >= rot){
                rightBack.setPower(0);
            }
            if (leftFront.getPower() == 0 && leftBack.getPower() == 0 && rightFront.getPower() == 0 && rightBack.getPower() == 0){
                stop = true;
            }
        }
    }

    public void forwardDist(double power, double targetDistance){
        resetDrivetrain();


        stop = false;

        if (!stop) {
            forward(power);

        }
        while (!stop) {
            LFdistance = Math.abs(leftFront.getCurrentPosition()/1680.0 * Math.PI * diameter);
            LBdistance = Math.abs(leftBack.getCurrentPosition()/1680.0 * Math.PI * diameter);
            RFdistance = Math.abs(rightFront.getCurrentPosition()/1680.0 * Math.PI * diameter);
            RBdistance = Math.abs(rightBack.getCurrentPosition()/1680.0 * Math.PI * diameter);
            if (LFdistance >= targetDistance){
                leftFront.setPower(0);
            }
            if (LBdistance >= targetDistance){
                leftBack.setPower(0);
            }
            if (RFdistance >= targetDistance){
                rightFront.setPower(0);
            }
            if (RBdistance >= targetDistance){
                rightBack.setPower(0);
            }
            if (leftFront.getPower() == 0 && leftBack.getPower() == 0 && rightFront.getPower() == 0 && rightBack.getPower() == 0) {
                stop = true;
            }
        }
    }

    public void moveDist(DcMotor motorName, double power, double ticks) {
        motorName.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorName.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        stop = false;

        if (!stop) {
            motorName.setPower(power);
        }
        while (!stop) {
            distance = Math.abs(motorName.getCurrentPosition());
            if (distance >= ticks) {
                motorName.setPower(0);
                stop = true;
            }
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }
}
