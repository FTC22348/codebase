/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object
 * Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia
 * license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow Object Detection")

public class Reapr_TFOD_Webcam extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as
     * an "asset",
     * the OpMode must to load it using loadModelFromAsset(). However, if a team
     * generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be
     * loaded using loadModelFromFile()
     * Here we assume it's an Asset. Also see method initTfod() below .
     */

    private static final String TFOD_MODEL_FILE = "model_reapr_ftcml_1.tfile";

    private static final String[] LABELS = {
            // Aarush - please insert the Reapr Teachable Machine classes here
            "0 Red",
            "1 Green",
            "2 Blue"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string
     * below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and
     * will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the
     * Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they
     * contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     * ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AcJWPiP/////AAABmRv0x6fBjk9rr2cVNe/65PMBQNZEE0OHUmJE6gJQfJrlvCAt/5rKCJ/7Sz9zJxCm+GXOmSArVpbJfrEOkQF9nRfrq3rkh8xHnSKc4tIl2KT7x9s2ev8d8S/mzJ+1NjqV7CBPVS7dFUqzcTgoqnuZgUJG/pzAFgCBJdkQdUDUMy8/qOdWuz8B8GthAKc5cmSFyBvvwk7y4Edmv/pqIwLiwP+M2H/13o/jySsQu6OctKGUSUMpvwX0Zd6BrmeaA9EVzAyWoURqzPkwQN1PBSUQVujQ6s1KZEDhhWs6EDzrcL66P35+7GVNSWEJYFTzdxeOjFNyv/tl5SoaKK0my47Fxid2Ta0Lm/OZx991/+kUBBga";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the
     * Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the
     * TensorFlow Object
     * Detection engine.
     */

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 840; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    /*************************/
    // Hardware Map
    /************************/

    // On port:
    double clawPosition = 0.0;
    final double clawSpeed = 0.05;// change to 100th when button is hold
    final double clawMinRange = 0.0;
    final double clawMaxRange = 0.55;
    boolean isSlowMode = false;
    double dividePower = 1.0;

    boolean isParallelMode = true;

    double frontLeftPower = 0.0;
    double backLeftPower = 0.0;
    double frontRightPower = 0.0;
    double backRightPower = 0.0;

    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, elevatorMotorLeft, elevatorMotorRight;
    Servo claw;

    public void ReaprHardware() {

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Port 0
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Port 1
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Port 2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Port 3

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Elevator Motors
        DcMotor elevatorMotorLeft = hardwareMap.dcMotor.get("elevatorMotorLeft");
        DcMotor elevatorMotorRight = hardwareMap.dcMotor.get("elevatorMotorRight");

        // Claw Motors (Servo)
        Servo claw = hardwareMap.servo.get("reaprClaw");// name of server on control

    } // End ; Call this in runOpMode

    public void red() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Forward
        encoderDrive(DRIVE_SPEED, -16, -16, -16, -16, 200.0);
        // 200 second timeout, no need for it yet

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Going left (functions are for going right because robot going BACKWARDS)
        encoderDrive(DRIVE_SPEED, 16, -16, -16, 16, 200.0);
        // 200 second timeout, no need for it yet
    } // Go forward 1.5 block then strafe left 1 block DICE 1/4

    public void green() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Forward
        encoderDrive(DRIVE_SPEED, -16, -16, -16, -16, 200.0);
        // 200 second timeout, no need for it yet
    } // Go forward 1.5 block DICE 2/5

    public void blue() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Forward
        encoderDrive(DRIVE_SPEED, -16, -16, -16, -16, 200.0);
        // 200 second timeout, no need for it yet

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Going right (functions are for going left because robot going BACKWARDS)
        encoderDrive(DRIVE_SPEED, -16, 16, 16, -16, 200.0);
        // 200 second timeout, no need for it yet
    } // Go forward 1.5 block then strafe right 1 block DICE 3/6

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we
        // create that
        // first.

        initVuforia();
        initTfod();
        ReaprHardware();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow
         * annotations visible.
         **/


        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a
            // lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or
            // 22").
            // If your target is at distance greater than 50 cm (20") you can increase the
            // magnification value
            // to artificially zoom in to the center of image. For best results, the
            // "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object
            // Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                String SignalDetected = "";
                String Red = "0 Red";
                String Green = "1 Green";
                String Blue = "2 Blue";
                sleep(2000); // wait 2 seconds
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                List<Recognition> recognitions = tfod.getRecognitions();
                for (Recognition recognition : recognitions) {
                    SignalDetected = recognition.getLabel();
                    if (SignalDetected == Red) {
                        telemetry.addData("Got Label: ", "%s", Red);
                        telemetry.update();
                        red();
                    }else if (SignalDetected == Blue) {
                        telemetry.addData("Got Label: ", "%s", Blue);
                        telemetry.update();
                        blue();
                    }else if (SignalDetected == Green) {
                        telemetry.addData("Got Label: ", "%s", Green);
                        telemetry.update();
                        green();
                    }else{
                        telemetry.addData(">", "ITS ALL AARUSH's FAULT");

                    }
                }

                tfod.deactivate();
            }
        }
    }// end of runOp() mode

    /**
     * Initialize the Vuforia localization engine.
     */



    public void encoderDrive(double speed, double leftInches, double rightInches, double leftBackInches,
            double rightBackInches, double timeoutS) {

        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE); // This was
        // connected on the expansion hub, it needs to be reversed

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = motorBackLeft.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = motorBackRight.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            motorFrontLeft.setTargetPosition(newLeftTarget);
            motorFrontRight.setTargetPosition(newRightTarget);
            motorBackLeft.setTargetPosition(newLeftBackTarget);
            motorBackRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both
            // motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when
            // EITHER motor hits
            // its target position, the motion will stop. This is "safer" in the event that
            // the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the
            // robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); // optional pause after each move
        }
    }

    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the
         * Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1"); // Has to be the hardware maped Reapr
        // webcam

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android
        // Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the
        // Robot Controller's FLASH.
        // tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);

    }

}
