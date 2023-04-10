# Bot-Mitzvah

2023 Team 1515 Auto Base

## Setting up module offsets

> Before setting up module offsets ensure each offset is set to `-Math.toRadians(0.0)` and that code is deployed to the
> robot. This must be done each time offsets are determined.
1. Either turn the robot on its side or put it up on a cart with the wheels exposed.

> By default, module sensor information is displayed in the `Drivetrain` tab on ShuffleBoard.
2. Rotate each module so the bevel gear on the sides of each wheel are pointing to the robot's left.
> When aligning the wheels they must be as straight as possible. It is recommended to use a long straight edge such as
> a piece of 2x1 in order to make the wheels straight.
3. Record the angles of each module using the reading displayed on the dashboard.

4. Set the values of the `*_MODULE_STEER_OFFSET` constants in `Constants` to `-Math.toRadians(<the angle you recorded>)`
5. Re-deploy and try to drive the robot forwards. All wheels should stay parallel to each other.
6. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel 
that is spinning in the incorrect direction. (I.e. `-Math.toRadians(<angle> + 180.0))`)
