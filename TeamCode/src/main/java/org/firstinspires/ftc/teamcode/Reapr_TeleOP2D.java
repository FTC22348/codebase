package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Reapr TeleOp 2 Drivers")


public class Reapr_TeleOP2D extends LinearOpMode{

    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, elevatorMotorLeft, elevatorMotorRight;
    Servo claw;
    // On port:
    double clawPosition = 0.0;
    final double clawSpeed = 0.05;// change to 100th when button is hold
    final double clawMinRange = 0.0;
    final double clawMaxRange = 0.55;
    boolean isSlowMode = false;
    double dividePower=1.0;

    boolean isParallelMode= true;

    double frontLeftPower = 0.0;
    double backLeftPower = 0.0;
    double frontRightPower = 0.0;
    double backRightPower = 0.0;

   /*************************/
    // Hardware Map
   /************************/
    public void ReaprHardware(){

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


    @Override
    public void runOpMode() throws InterruptedException {

        // Call Hardaware Map Function
        ReaprHardware();

        // Create Thread Instance
        Thread reaprThread = new ReaprDriveThread();


        telemetry.addData("Mode", "Waiting");
        telemetry.update();

        //Wait for Satrt to be Pressed
        waitForStart();


        //Start Parallel Thread
        reaprThread.start();


        //if (isStopRequested()) return;

        try
        {
          while (opModeIsActive()) {


              // Control Speed
              if(isSlowMode){
                  dividePower=1.5;
              }else{
                  dividePower=1.0;
              }

              if(gamepad1.left_stick_button){
                  if(isSlowMode){
                      isSlowMode=false;
                      sleep(500);
                  }else{
                      isSlowMode=true;
                      sleep(500);
                  }
              }

              telemetry.addData("claw", "%.2f", clawPosition); //displays the values on the driver hub
              telemetry.update();

              /*******************************/
              // Drive Wheels Movement
              /*******************************/


                  // Mecccanum controls
                  double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                  double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                  double rx = gamepad1.right_stick_x;

                  telemetry.addData("X: ", "%.2f", x);
                  telemetry.addData("Y: ", "%.2f", y);
                  telemetry.update();

                  // Denominator is the largest motor power (absolute value) or 1
                  // This ensures all the powers maintain the same ratio, but only when
                  // at least one is out of the range [-1, 1]
                  double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                  double frontLeftPower = (y + x + rx) / denominator / dividePower; //Positive rotation results in forward & right motion
                  double backLeftPower = (y - x + rx) / denominator / dividePower; //Positive rotation results in forward & left motion
                  double frontRightPower = (y - x - rx) / denominator / dividePower; //Positive rotation results in forward & left motion
                  double backRightPower = (y + x - rx) / denominator / dividePower; //Positive rotation results in forward & right motion

                  telemetry.addData("Moter Front Left Power: ", "%.2f", frontLeftPower);
                  telemetry.addData("Moter Back Left Power: ", "%.2f", backLeftPower);
                  telemetry.addData("Moter Front Right Power: ", "%.2f", frontRightPower);
                  telemetry.addData("Moter Back Right Power: ", "%.2f", backRightPower);
                  telemetry.update();



                  motorFrontLeft.setPower(frontLeftPower); // Get Power from Thread
                  motorBackLeft.setPower(backLeftPower); // Get Power from Thread
                  motorFrontRight.setPower(frontRightPower); // Get Power from Thread
                  motorBackRight.setPower(backRightPower); // Get Power from Thread


              idle();

          } // end of opModeIsActive() while loop

        }
        catch (Exception e) {telemetry.addData("Reapr Opmode Fault :", "e");telemetry.update();}

        // Stop Driving Thread
        reaprThread.interrupt();
        isParallelMode = false;

    }// End runOpMode


    // Create a thread class

    private class ReaprDriveThread extends Thread

    {

        public ReaprDriveThread() {
            this.setName("DriveThread");
        }

        @Override
        public void run() {

            while (!isInterrupted()){

                /********************************/
                // Drive Claw
                /********************************/

                    // Servo Controls

                    if (gamepad2.b) // Down
                        clawPosition += clawSpeed;
                    else if (gamepad2.x) // Up
                        clawPosition -= clawSpeed;

                    clawPosition = Range.clip(clawPosition, clawMinRange, clawMaxRange);
                    claw.setPosition(clawPosition);


                /**************************************/
                // Elevator Movement
                /*************************************/


                    if (gamepad2.a) { // Move down
                        elevatorMotorLeft.setPower(0.5);
                        elevatorMotorRight.setPower(-0.5);
                    }
                    elevatorMotorLeft.setPower(0);
                    elevatorMotorRight.setPower(0);

                    if (gamepad2.y) { // Move up
                        elevatorMotorLeft.setPower(-0.7);
                        elevatorMotorRight.setPower(0.7);


                    elevatorMotorLeft.setPower(0);
                    elevatorMotorRight.setPower(0);
                }
                /**************************************/
               idle();
            } // end while for is interupted

        }
    } // End Thread Class

}// End Master Class
