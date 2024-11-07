# simplesim!

![docker ci build](https://github.com/nabeelsherazi/simplesim/actions/workflows/docker-image.yml/badge.svg)

simplesim is a little ROS2 + SFML exercise in implementing a good pure pursuit controller.

![image](https://github.com/user-attachments/assets/b7e99e1b-69a5-4958-a23f-446d053a0bfc)

This project is designed to have an emphasis on being clear and readable, so you can learn to implement one too!

## Build and Run (local)

Have [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installed and sourced.

```sh
cd ~/src/ros2_ws     # or wherever your colcon workspace is
git clone git@github.com:nabeelsherazi/simplesim src/simplesim
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

Run:

```sh
. install/setup.bash
ros2 launch src/simplesim/launch/all.launch # properly
# or, with all topics un-namespaced
ros2 run simplesim simplesim_node
```

The launch file can also spin up a [Foxglove Bridge](foxglove.dev) node for you with `--foxglove`

## Build and Run (Docker)

```sh
xhost +local:docker # Let docker
docker build -t simplesim:latest .
docker run --rm --device=/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix simplesim:latest
```

## About

simplesim models a quadrotor with *double integrator dynamics* in 2D. Double integrator dynamics means that kinematic motion is completely determined by the instantaneous acceleration at any given point in time. The quadrotor can accelerate instantaneously in any direction at any point in time. This makes control problems much simpler.

## About the vehicle simulation

The quadrotor simulation is modeled in a decoupled way from the rest of the program, meaning that you can drop in your own controller to try controlling it! It can be controlled in either velocity or acceleration mode, meaning it will accept a velocity and instantaneously move at that velocity (simpler!) or accept an acceleration an instantaneously start accelerating at that rate (more realistic).

## About the controller

The built-in controller is a cascaded PD controller, which, given a goal, commands the quadcopter to accelerate. The setting of the goal is where pure pursuit comes in. When you click on the screen to add a waypoint to the path, the controller uses a pure pursuit algorithm to determine the goal (setpoint) for the cascaded PD controller. Thus you get smooth path following.

## About pure pursuit control

Pure pursuit control was first illustrated in this [1992 paper by R. Craig Coulter](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf). While the paper is readable, it leaves a lot of things unclear, which this implementation should illuminate.

The basic idea of pure pursuit is that you have a path you would like your vehicle to follow. Like how when driving a car you don't stare at exactly what's in front of you, but rather ahead and towards where you want to go, pure pursuit puts a "carrot" in front of the vehicle, following the path, that allows the vehicle to make smooth motions.

The way the algorithm works is like this:

1. Define some lookahead distance $L_D$. When we are on the path, we ideally want to track a point that is always exactly one lookahead distance away from us (like a stick we are holding with a carrot on it -- it should sweep a circle around the vehicle).
2. Start by finding the nearest point on the path to the vehicle. If we're far away from the path, this is our best lookahead point.
3. Find the intersections of our lookahead radius with the path. The best lookahead point is the one furthest along the path.

There's a gotcha though. As we're looking for intersections of the path with the lookahead circle, we could catch paths that are far ahead of where we are now. So we limit ourselves to either the current segment we're on, *or* any following segments so long as their starting point is within our lookahead radius. This prevents us from accidentally jumping ahead.

The math works like this. Let the current path segment be between waypoints $\vec{p_1}$ and $\vec{p_2}$, and our current position be $\vec{p_v}$.

If we're far away from the path, our best waypoint is the point on the segment closest to the vehicle, which we get by taking the projection of the vehicle position onto the segment.

In code that's:

```cpp
// Vector from p1 to p2
auto p1 = waypointList[currentWaypointIndex];
auto p2 = waypointList[currentWaypointIndex + 1];

sf::Vector2f segment = p2 - p1;
sf::Vector2f currentToP1 = currentPosition - p1;
float segmentLength = simplesim::norm(segment);
// Find the projection point
float t = simplesim::dot(currentToP1, segment) / (segmentLength * segmentLength);

// Clamp t to [0, 1] to stay within segment bounds
t = simplesim::clamp(t, 0.0F, 1.0F);

// Initial best lookahead point is the projection point
currentSetpoint = p1 + t * segment;
```

If we're close to the path, we need to find the lookahead point. We can do this by finding the intersections of the lookahead circle with the path. The lookahead circle is centered at the vehicle and has radius $L_D$.

In vector form, the equation of a circle is:

$$
\|\vec{p} - \vec{c}\|^2 = r^2
$$

where $\vec{p}$ is a point on the circle, $\vec{c}$ is the center of the circle, and $r$ is the radius of the circle. In our case, the center of the circle is the vehicle position, and the radius is $L_D$. The point $\vec{p}$ is the lookahead point we're looking for.

If we parameterize our line segment as:

$$
\vec{d} = \vec{p_2} - \vec{p_1} \\
\vec{p}(t) = \vec{p_1} + t\vec{d}
$$

then we can substitute this into the equation of the circle to find points that both lie on our segment and in the circle.

$$
\| \vec{p_1} + t\vec{d} - \vec{c} \|^2 = r^2
$$

Let $\vec{f} = \vec{p_1} - \vec{c}$. Then this becomes:

$$
\| \vec{f} + t\vec{d} \|^2 = r^2
$$

and expands to:

$$
(\vec{f} + t\vec{d}) \cdot (\vec{f} + t\vec{d}) = r^2
$$

which is (collecting the dot products):

$$
\vec{f} \cdot \vec{f} + 2t(\vec{f} \cdot \vec{d}) + t^2(\vec{d} \cdot \vec{d}) = r^2
$$

and can be written as a quadratic equation in $t$:

$$
t^2(\vec{d} \cdot \vec{d}) + 2t(\vec{f} \cdot \vec{d}) + (\vec{f} \cdot \vec{f} - r^2) = 0
$$

If we let:

$$
a = \vec{d} \cdot \vec{d}
$$
$$
b = 2\vec{f} \cdot \vec{d}
$$
$$
c = \vec{f} \cdot \vec{f} - r^2
$$

then we can solve the quadratic equation for $t$. As you'll probably remember, the quadratic formula tells us how many solutions we'll have based on whether the discriminant is positive, zero, or negative.

One very nice trick of solving in this parameterized form it that *it makes it very easy for us to tell which intersections are further along the path*. Since the segment is parameterized from beginning to end, we take the largest t as our lookahead point. We just convert it back into a point with $\vec{p}(t)$.

```cpp
// Find the lookahead point along the path.

// If we haven't reached the first waypoint yet, the segment is from the first waypoint to the second.
// Otherwise, the segment is from the last waypoint to the current one.
auto segmentStartIndex = currentWaypointIndex == 0 ? 0 : currentWaypointIndex - 1;

for (int i = segmentStartIndex; i < waypointList.size(); i++) {
    p1 = waypointList[i];
    p2 = waypointList[i + 1];

    // After the current segment, we only check following segments if their start point is within the lookahead distance circle
    if (i > segmentStartIndex && !simplesim::pointWithinCircle(currentPosition, lookaheadDistance, p1)) {
        break;
    }

    auto intersections =
        simplesim::intersectCircleAndLineParameterized(currentPosition, lookaheadDistance, p1, p2);

    if (!intersections.empty()) {
        auto furthestIntersection = std::max_element(intersections.begin(), intersections.end());
        currentSetpoint = p1 + *furthestIntersection * (p2 - p1);
    }
}
```

### Pure pursuit with adaptive lookahead

One problem with pure pursuit control is that when traveling in long straight lines, you want to be able to speed up. With a fixed lookahead distance, your carrot is a fixed radius from the vehicle, and thus the PID controller will have some maximum output command that won't be exceeded. Adaptive pure pursuit is illustrated in this [2007 paper by S. Campbell](http://hdl.handle.net/1721.1/42301). The simple idea is to scale the lookahead distance by the current speed. By providing a minimum and maximum lookahead, and a lookahead gain $K_L$, the lookahead distance to use is:

$$
L_D = \min(L_{max}, \max(L_{min}, L_{min} + K_L \| v \|))
$$

### In an alternative world where the dynamics are not so simple

Given the current vehicle position and an (x, y) goal we would like to go to. Let's find the curvature required to hit that point. Curvature is defined as $\kappa = 1/r$, where r is the radius of a circle. The larger the radius of the circle, the smaller the curvature -- a straight line is a circle of infinite radius.

The straight line of length $l$ from our position to (x, y) makes a chord of a circle, but the problem is that there are *infinitely* many circles that can fit that arc. So we add one more constraint, choosing a circle such that its center lies on the x axis (assuming our vehicle position is at the origin). Then, from our vehicle to the center of the circle is distance R, and the center is at position $(R, 0)$.

The equation of a circle must hold. That means:

$$
(x - R)^2 + y^2 = R^2 \\
$$

$$
R = \frac{x^2 + y^2}{2x}
$$

$$
\begin{equation}
\frac{1}{R} = \kappa = \frac{2x}{x^2 + y^2} = \frac{2x}{l^2}
\end{equation}
$$

Cool! If our quadrotor didn't have double integrator dynamics, we would use this as our commanded steering angle!

Now in our case, 

## About the wind and drag model

The wind vector in this simulation is modeled as a 2D vector with randomly walking components. The independent random walks really work well for simulating wind. At each time step, the current wind vector is treated like an acceleration, meaning that the `windIntensity` parameter (very) roughly is analogous to the maximum jerk the wind can apply to the vehicle.

Drag is modeled like this: the real equations are quadratic in velocity, but since the Reynolds number $Re$ also depends on velocity, there's also a linear regime when the $Re$ is decreasing like $1/v$. At some point, however, it plateaus, and you enter the quadratic regime, usually around Re=1000. For most objects, this happens at very slow speeds, like less than 0.5 m/s, meaning that it would be best to only use quadratic drag technically. However, this simulation models the linear regime and a faux transition anyway, because starting from quadratic drag can look very unphysical.

## To do:

[ ] [Regulated Pure Pursuit](https://arxiv.org/abs/2305.20026)

[ ] I think there is a minor bug in how I select the segment to use for the initial pure pursuit guess. It initializes with `segment = waypointList[currentWaypointIndex + 1] - waypointList[currentWaypointIndex]` which is correct when we have not reached our very first waypoint yet but incorrect after that. This also causes the behavior where once we reach the second to last endpoint, the lookahead is set directly to the last endpoint.

[ ] Test and verify that all on-the-fly parameter changing works

[ ] Add faster than realtime support for simulations
