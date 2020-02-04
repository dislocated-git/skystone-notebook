# Programming and Control

One of the key features implemented in our OPModes is the control of our holonomic drivetrain. As a relatively new team, it was quite a challenge to get our head around the math behind controlling a robot with such drivetrain design.  
Here's an outline of what we want to achieve with our holonomic drivetrain:

* Driving straight without turning at any angle
* Driving in a straight line consistenly without overshooting
* Changing the heading of the robot by rotating on the spot

To achieve this, we decided to use the stick of the controller as it provides us with both x and y components for our vector of movement, as well an easy way for the driver to precisely and quickly control the direction the robot is heading in. For our autonomous, we use methods that mimic this input.

## First prototype

So to start with we created a some code to get the robot moving in four directions:

``` java
if (gamepad1.dpad_up || gamepad1.dpad_down) {
    int magnitude = (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0);
    frontLeftPower = magnitude;
    frontRightPower = magnitude;
    backLeftPower = magnitude;
    backRightPower = magnitude;
}
if (gamepad1.dpad_left || gamepad1.dpad_right) {
    int magnitude = (gamepad1.dpad_left ? 1 : 0) + (gamepad1.dpad_right ? -1 : 0);
    frontLeftPower = -magnitude;
    frontRightPower = magnitude;
    backLeftPower = magnitude;
    backRightPower = -magnitude;
}

frontLeftDrive.setPower(frontLeftPower);
frontRightDrive.setPower(frontRightPower);
backLeftDrive.setPower(backLeftPower);
backRightDrive.setPower(backRightPower);
```

Although this code allows to switch the direction of movement rather quickly, we felt like this would limit us from doing any precise movements, so we decided to do some research on holonomic motion at an angle. We decided to implement turning in place for more precise control. However, after our orientation has been changed, we need to remain moving in 4 cardinal directions. To do this, we did a little bit of trigonometry to determine the power of each motor when driving at an angle.

```java
// Get our orientation using built-in imu
Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

...

// Get the Z component of the orientation
float angle = Math.abs(angles.firstAngle);

// Forwards and backwards
if (gamepad1.dpad_up || gamepad1.dpad_down) {
    int magnitude = (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0);
    frontLeftPower = magnitude * Math.sin(angle);
    frontRightPower = magnitude* Math.sin(angle+90);
    backLeftPower = magnitude* Math.sin(angle+90);
    backRightPower = magnitude* Math.sin(angle);
}
// Left and right
if (gamepad1.dpad_left || gamepad1.dpad_right) {
    int magnitude = (gamepad1.dpad_left ? 1 : 0) + (gamepad1.dpad_right ? -1 : 0);
    frontLeftPower = -magnitude * Math.sin(angle+90);
    frontRightPower = magnitude* Math.sin(angle);
    backLeftPower = magnitude* Math.sin(angle);
    backRightPower = -magnitude* Math.sin(angle+90);
}
```

After testing this approach we decided that it was still not enough for moving around the field with a high degree of freedom, so we decided to take this one more step further and drive in all directions at any angle.

## Driving at any angle

To be able to drive at any angle in all possible directions we had to reassign our input to the stick, as it allowed greater control over the vector we want to head in and helpfully provides it's data as x and y components, thanks to the FTC library.

```java
float x = -gamepad1.left_stick_x;
float y = -gamepad1.left_stick_y;

frontLeftPower = y + -x;
frontRightPower = y + x;
backLeftPower = y + x;
backRightPower = y + -x;
```

Since the components are split for us, all we have to do is add them to drive in the direction of the vector provided. One issue that arises when using this method is the difference in magnitudes of vectors when driving in certain directions. This was solved previosly while doing the trigonometry method as some trig values gave us higher speeds than others, so we repurposed that code.

```java
double max = Math.abs(findMax(frontLeftPower, frontRightPower, backLeftPower, backRightPower));

if (max != 0) {
    frontLeftPower = (frontLeftPower / max) * basePower;
    frontRightPower = (frontRightPower / max) * basePower;
    backLeftPower = (backLeftPower / max) * basePower;
    backRightPower = (backRightPower / max) * basePower;
}
```

Ok, so now our robot can drive in all directions and rotate around itself. This is basically enough for our usecase, but we decided to also implement a toggable PID system that would maintain a heading. This can be used in autonomous for driving and TeleOP in case we have a failure that affects the movement.

This is some logic behind the PID:

* If robot is turning around itself - the pid is off
* If the robot is changing it's vector of movement - the pid is off
* Once the robot acquires a vector of movement the pid maintains it until user input is given
* The pid applies a correction value to the motor power when on

Here's a code snippet demonstrating using the PID

```java
if (!turning && lastTurning) {
    resetAngle();
}

...

if (!turning) {
    // Use PID with imu input to drive in a straight line.
    correction = pidDrive.performPID(getAngle());
}
else{
    correction = 0;
}
```
