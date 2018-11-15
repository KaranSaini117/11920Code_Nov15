/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "LowerDepotSample", group = "11920")
//@Disabled
public class LowerDepotSample extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Motor Setup
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private DcMotor arm = null;
    private Servo marker = null;
    double distance;
    double LFdistance;
    double LBdistance;
    double RFdistance;
    double RBdistance;
    double diameter = 6;
    boolean stop = false;
    int tick;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AczxNeP/////AAABmVQoW+m1DkrLg5u3bSo+nSs4OPM1TQ7fq5fOkT9rUJXW26EZB8WpJsEXFiKCzduaSQt60AilIaqFARDn0DhF5m/fhKcBMha9airEGQ0Z2N7EBnvVBgODphonKiXsw+34/uldsPcgtiXLo2hEk04NTerCk1mVAP3hICYReLDXNN2g+zSKMYkOPA8aZa1Y/LIKQkhY/+Skmy0XEXrbUXp2RaGYOyH9dIQzuwdvuekmc0s8Lz73zSalHSB/j03ny1yvAFeyYUn6CiJbTU0cOyxjJ2h/QDCabeVUPYe0zPKWY8uEcnBWNmwzQKcSeyOuYvMTTrTzHpP7PM3ycL64KqKj0Rko5t6QnmPRyAraIgzdp14A";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        lift = hardwareMap.get(DcMotor.class,"lift");
        arm = hardwareMap.get(DcMotor.class,"arm");
        marker = hardwareMap.get(Servo.class, "marker");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setPower(0);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }


            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");

                                    // Run Center Auton once detected

                                    // Needs to lower
                                    /*arm.setPower(-0.2);

                                    moveDual(lift,1,11200);

                                    lift.setPower(0);
                                    arm.setPower(0);

                                    sleep(500);

                                    turnDeg(-0.4,0.15);

                                    sleep(100);

                                    forwardDist(0.4,30);

                                    sleep(100);

                                    turnDeg(0.4,0.55);

                                    sleep(100);

                                    forwardDist(0.4,30);

                                    sleep(100);

                                    turnDeg(0.4,1.35);

                                    marker.setPosition(0.65);

                                    sleep(1000);

                                    marker.setPosition(0);

                                    forwardDist(0.4,94);

                                    sleep(30000);*/



                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
    Not recommended to make rotations any larger. It will become inaccurate.
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
            if (leftFront.getPower() == 0 && leftBack.getPower() == 0 && rightFront.getPower() == 0 && rightBack.getPower() == 0) {
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

    public void moveDual(DcMotor motorName, double power, double ticks) {
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
            if (Math.abs(arm.getCurrentPosition()) >= 700){
                arm.setPower(0);
            }
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }
}