/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@TeleOp(name = "Sensor: REVColorDistance", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class wallTracking extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    DistanceSensor rangeSensor;

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

    @Override
    public void runOpMode() {


        // get a reference to the distance sensor that shares the same name.
        rangeSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", rangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.update();
        }
    }
    private double TOLERANCE = 2;
    private double GAIN = 0.05; // Needs tuning to find right scaling factor. Start low like 0.023 then increase or at least find a value in which your robot stops oscillating sharply.

    //drive along wall

    //comment

    public void driveAlongWall()
    {
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;
        double error = rangeSensor.getDistance(DistanceUnit.INCH) - 15;
        if(error < TOLERANCE)  //The robot is too close to the wall
        {
            telemetry.addLine("Too close");
            telemetry.update();
            /*
            leftFrontPower = (GAIN * error);
            leftBackPower = GAIN * error;
            rightFrontPower = (GAIN * error) + GAIN;
            rightBackPower = (GAIN * error) + GAIN;
            */
        } else if(error > TOLERANCE)  //The robot is too far away
        {
            telemetry.addLine("Too far");
            telemetry.update();
            /*
            leftFrontPower = GAIN * error;
            leftBackPower = -GAIN * error;
            rightFrontPower = -GAIN * error;
            rightBackPower = GAIN * error;
            */
        } else {
//            leftFrontPower = 0.5;
//            leftBackPower = 0.5;
//            rightFrontPower = 0.5;
//            rightBackPower = 0.5;
            telemetry.addLine("close enough");
            telemetry.update();

        }
        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }
}
