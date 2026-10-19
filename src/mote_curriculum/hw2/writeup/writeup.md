<!---
If you're new to markdown, check out this reference: https://www.markdownguide.org/basic-syntax/
-->

# Writeup

## Q0. Investigating Transforms in Foxglove (10 pts)

**Q0.1:**
![alt text](../images/base_link_axis.png)
Rover moves forward along the x (red) axis. As pictured above where the left wheel is to the left of the base_link axis and the right wheel is to the right and thus the x axis is pointing along the front.


![alt text](../images/left_wheel.png)
![alt text](../images/right_wheel.png)
When the rover moves forward, at the point of contact there is assumed to be a no slip condition and thus v_contact = 0 while v_axle = v and v_top = 2v. Thus from the left wheel's perspective the wheel must be moving counterclockwise so that the top is pointing to the left while looking down the Z axis. From the right wheel's perspective the wheel must be moving counterclockwise as well in order to have leftward motion at the top of the wheel when looking down the Z axis. Since CCW is positive in a right handed coordinate system, both wheels have a positive rotation direction when the rover moves forward. 

## Q1. Identifying a Kinematic Model (20 pts)

**Q1.1:** 

<!---
You can use LaTeX syntax like so: $a_{example} = b_{example} + c_{example}$.
You can use $\upsilon$ for linear velocity and $\omega$ for angular velocity.

If you prefer to write out your work, you can include it in an image as follows:
![image description](./image_path.png)
-->

Linear velocity:
$$v = \frac{v_r + v_l}{2}$$

Angular velocity:
$$\omega = \frac{v_r - v_l}{b}$$

$$2v = v_r + v_l$$
$$b\omega = v_r - v_l$$

Solving for $v_r$:
$$v_l = 2v - v_r$$
$$b\omega = v_r - 2v + v_r$$
$$ v_r - v = \frac{b\omega}{2}$$
$$\boxed{v_r = \frac{b\omega}{2} + v}$$

Substituting:
$$v_l = 2v - \frac{b\omega}{2} - v$$
$$\boxed{v_l = v - \frac{b\omega}{2}}$$

**Q1.2:**

$$\dot{\phi} = \frac{v}{r} = \frac{2v}{d}$$
$$\boxed{\dot{\phi}_l = \frac{2v_l}{d}}$$
$$\boxed{\dot{\phi}_r = \frac{2v_r}{d}}$$

**Q1.3:** 

Linear velocity:
$$\boxed{v = \frac{v_r + v_l}{2}}$$

Angular velocity:
$$\boxed{\omega = \frac{v_r - v_l}{b}}$$

## Q2. Inverse Kinematics (20 pts)

**Q2.2:** 

**Q2.3:** 

## Q3. Forward Kinematics (20 pts)

**Q3.2:** 

## Q4. Calculating an Odometry Solution (30 pts)

**Q4.3:** 


## Feedback

**How much time did you spend on this assignment?** 

**Do you have any feedback for this assignment?**
