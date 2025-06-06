Ultimate Guide: From Embedded Systems to Autonomous Robotics
Introduction: This guide provides a comprehensive roadmap from low-level embedded systems (programming bare-metal hardware) up to high-level autonomous driving and robotics systems. We will cover fundamental concepts, tools, and languages at each layer of this technological stack – including hardware programming, sensors/actuators, control theory, robotics algorithms, machine learning, and autonomous vehicle architecture. The content is aimed at advanced practitioners and researchers, emphasizing why each topic matters and the mathematical foundations behind them. For beginners, note that proficiency in programming (particularly C/C++ for embedded and Python for higher-level robotics/AI) is essential
erc-bpgc.github.io
. Below, we break down the journey into logical sections, each building on the previous.
Embedded Systems Fundamentals (Bare Metal)
An embedded system is essentially a specialized computer integrated into a larger system, dedicated to specific functions (unlike a general-purpose PC). It typically includes at least one microcontroller or microprocessor and is optimized for reliability, cost, size, and power efficiency
github.com
. In other words, an embedded system is a purpose-built computing system within a device – for example, the controller inside a microwave or a car’s engine control unit
github.com
. Bare-metal programming refers to writing software that runs directly on the hardware without any operating system abstraction
en.wikipedia.org
. In a bare-metal (or “bare machine”) setup, your code can interact directly with memory-mapped I/O registers and CPU instructions to control hardware. This yields highly efficient use of memory and CPU since there’s no OS overhead
en.wikipedia.org
. Bare-metal programs are common in resource-constrained embedded devices where maximal performance and minimal latency are required
en.wikipedia.org
. However, the trade-off is complexity: without an OS, the developer must manage the hardware resources (timers, I/O, interrupts, etc.) and even write basic scheduling or drivers manually
en.wikipedia.org
. Debugging can be challenging (no standard output or process isolation), and one must set up toolchains to flash and test code on the target hardware
en.wikipedia.org
. Typical languages for bare-metal development are low-level ones like C or C++, and often assembly, since they allow direct memory access and precise timing control
en.wikipedia.org
. For instance, you might write an ISR (Interrupt Service Routine) in C to handle a hardware timer tick, or even inline assembly to toggle a GPIO pin with nanosecond accuracy. Understanding computer architecture is crucial: you should be familiar with CPU registers, memory (flash, RAM), and peripherals (UART, SPI, I2C controllers, etc.). A basic example of bare-metal programming is configuring a microcontroller’s registers to blink an LED without any OS – essentially an infinite loop (sometimes called a superloop
en.wikipedia.org
) that polls inputs and toggles outputs. Microcontrollers vs. Microprocessors: A microcontroller unit (MCU) is an all-in-one chip that contains a CPU plus integrated memory and I/O peripherals, designed to run standalone without an OS
ibm.com
. MCUs are usually used for dedicated tasks (like reading a sensor and controlling an actuator) and run at modest clock speeds with limited memory, but they are extremely cost-effective and low-power
ibm.com
. A microprocessor unit (MPU), by contrast, is usually just a powerful CPU that relies on external memory and peripherals – like the CPUs in your laptop or Raspberry Pi. Microprocessors can run complex operating systems (Linux, Windows) and handle high-performance tasks, whereas microcontrollers excel at real-time control for specific applications (e.g. in IoT devices or automotive controllers)
ibm.com
ibm.com
. One consequence of this difference is architecture: many microcontrollers use Harvard architecture (separate instruction and data buses) for speed, while most microprocessors use von Neumann architecture (shared memory bus) for flexibility
ibm.com
ibm.com
. In practice, as an embedded engineer you might use an 8-bit AVR or a 32-bit ARM Cortex-M MCU to handle sensor reading and motor control on a robot, while a more powerful processor (like an ARM Cortex-A or x86) runs high-level planning algorithms. Tools: Developing for embedded systems typically involves cross-compilation (compiling code on a PC to run on the target device) and hardware debugging. Some essential tools and components include:
Cross-Compiler Toolchains: e.g. GNU Arm Embedded GCC for compiling C/C++ into firmware binary for ARM microcontrollers.
Debuggers: Hardware debuggers like JTAG or SWD probes (e.g. ST-Link, J-Link) allow stepping through code on the device, and logic analyzers or oscilloscopes help in inspecting signals.
Integrated Development Environments (IDEs): Popular IDEs for embedded include Keil μVision, IAR Embedded Workbench, and also general editors like VS Code with extensions (e.g. Cortex-Debug)
github.com
. Many use PlatformIO, a cross-platform build system and library manager that supports multiple frameworks and MCUs, to streamline development
github.com
.
Version Control & Build Systems: As projects grow, tools like Git, CMake, etc., become important to manage code and builds for different target boards or configurations.
Embedded development often starts with vendor-provided board support packages and HAL (Hardware Abstraction Layer) libraries which provide C functions to configure clocks, GPIOs, communication interfaces, etc. Beginners frequently start with platforms like Arduino (built on AVR/ARM MCUs) because it simplifies a lot of setup and has a vast collection of libraries. (Indeed, Arduino’s libraries and IDE use C/C++ under the hood and are great for learning basics
erc-bpgc.github.io
.) As you advance, you might work directly with MCU registers or use professional RTOS and frameworks as described next.
Real-Time Operating Systems and Embedded Software
As embedded projects grow more complex, running everything in a superloop (bare-metal) can become limiting – especially when you have tasks that need precise timing or should run independently. This is where Real-Time Operating Systems (RTOS) come in. An RTOS is a lightweight operating system optimized for predictable timing and concurrency in embedded systems
github.com
. Unlike general-purpose OSes, an RTOS is designed to ensure that critical tasks meet their deadlines with minimal jitter. This deterministic behavior is vital in domains like avionics, automotive, robotics, and medical devices, where a delayed response could be catastrophic
github.com
. RTOS Basics: An RTOS provides primitives like tasks/threads, semaphores, mutexes, and message queues for inter-task communication, all with minimal overhead. The scheduler in an RTOS can be priority-based preemptive (ensuring high-priority tasks run as soon as needed), and often supports features like time slicing and interrupt handling with priority inversion control. Key concepts include task priorities, interrupt service routines, and mutual exclusion to avoid race conditions. The benefit of an RTOS over bare-metal is that you can structure your firmware as concurrent tasks – for example, one task reads sensors periodically, another handles a user interface, and a high-priority task monitors for an emergency stop signal. The RTOS scheduler ensures each gets CPU time as appropriate and deadlines are met (e.g., a control loop running at 100 Hz). This makes the software more modular and easier to scale. Hard vs. Soft Real-Time: In hard real-time systems, missing a deadline is a total failure (e.g., an airbag sensor must trigger within a few milliseconds or it’s useless). In soft real-time systems, occasional misses are tolerable but undesirable (e.g., a video frame dropped). Many RTOSes are designed for hard real-time guarantees, offering features like fixed-priority scheduling (Rate Monotonic or Earliest Deadline First algorithms) and precise interrupt latency documentation. Examples of RTOS: Popular RTOSes in the embedded world include FreeRTOS, Zephyr, VxWorks, MQX, ThreadX, among others. For instance, FreeRTOS is open-source and widely used in IoT devices – it lets you create tasks and set their stack sizes and priorities easily, and has a tick interrupt for timekeeping
github.com
. Zephyr is another open-source RTOS (backed by the Linux Foundation) that is growing in popularity for its robust device driver support and multi-architecture compatibility
github.com
. These RTOSes typically run on microcontrollers with as low as tens of kB of RAM. Embedded Linux: On the higher end of embedded systems, you have boards like the Raspberry Pi or NVIDIA Jetson that run embedded Linux. Linux isn’t a real-time OS by default (unless using the PREEMPT_RT patch or Xenomai, etc.), but it provides the advantage of a rich ecosystem. In robotics, it’s common to use a hybrid approach: microcontrollers running bare-metal or RTOS code handle direct control of motors and sensors, while a more powerful processor running Linux handles complex computations and high-level planning. Embedded Linux allows you to use powerful frameworks (like ROS, discussed later) but typically cannot guarantee hard real-time behavior without special configurations. Safety-Critical Systems: In automotive and autonomous driving systems, there are often safety-critical components. These might use a specialized OS like QNX or an AUTOSAR-compliant real-time OS. QNX, for example, is a real-time POSIX OS often used in cars for its reliability and safety certification; it supports microkernel architecture, meaning even device drivers run in user space to reduce the chance a faulty driver can crash the system. These systems are designed for fault tolerance and often run on automotive-grade microprocessors. For instance, an autonomous vehicle might use an RTOS for the steering/braking control subsystem that must respond within milliseconds reliably
blackberry.qnx.com
, while running Linux for the AI perception subsystem that is less time-critical but very compute-intensive. In summary, whether to use bare-metal or an RTOS (or full OS) depends on the application’s complexity and timing requirements
github.com
. Many real projects mix approaches: time-critical loops in one microcontroller, and a higher-level computer orchestrating tasks and user interaction.
Sensors, Actuators, and Interfacing with Hardware
At the heart of any embedded or robotic system are sensors (which measure the real world) and actuators (which affect the real world). A successful engineer needs to know how to interface these correctly with microcontrollers, including understanding signals, conversion, and drivers. Microcontrollers and I/O: A microcontroller is the bridge between sensors and actuators in a robot’s electronics system
erc-bpgc.github.io
. It reads inputs from sensors (voltage, current, digital signals) and computes outputs to drive actuators, often in a continuous loop (the control loop). The connection typically involves signal conditioning: e.g., using analog-to-digital converters (ADC) to read analog sensor voltages, or pulse-width modulation (PWM) outputs to control motor speed. Microcontrollers come with a variety of communication peripherals – UART, SPI, I²C, CAN, etc. – which are used to communicate with smart sensors or other controllers. For instance, many sensors (like IMUs, temperature sensors, etc.) communicate via I²C or SPI bus, and an embedded system must implement those protocols to query the sensor data. Sensors: Sensors allow a system to sense its environment or internal state. They can measure all kinds of physical quantities. Some common sensors in robotics and autonomous systems include:
Position/Motion Sensors: e.g. rotary encoders on wheels or motors (to measure rotation angle or speed), gyroscope and accelerometer (often combined in an IMU – Inertial Measurement Unit – to measure orientation and acceleration), and GPS for global position.
Environmental Sensors: e.g. temperature sensors, pressure sensors, light sensors, etc., which might be needed for certain robotic tasks (or for system health monitoring).
Distance/Proximity Sensors: Ultrasonic sensors use sound waves to measure distance to the nearest object (popular in simple robots for obstacle avoidance). Infrared rangefinders can do short-range distance sensing or vision (like night-vision cameras use IR)
blackberry.qnx.com
. LiDAR (Light Detection and Ranging) uses laser beams to scan the environment and produce a high-resolution 3D point cloud of distances
blackberry.qnx.com
 – these are crucial in autonomous vehicles for mapping obstacles, though high-end 360° LiDARs are expensive. Radar uses radio waves to detect objects and measure their relative speed (Doppler effect); automotive radars operate at 24 GHz or 77 GHz and excel at long-range detection and speed measurement, unaffected by lighting or weather
blackberry.qnx.com
blackberry.qnx.com
.
Imaging Sensors: Cameras (monocular or stereo) are key sensors for autonomous systems, providing rich visual information. With multiple cameras, one can achieve a 360° view around a vehicle or depth perception via stereo vision
blackberry.qnx.com
. Cameras are used for object recognition, lane detection, etc., often in conjunction with advanced computer vision algorithms (discussed later in perception).
Specialized sensors: e.g. Magnetometer (compass), GPS (global positioning), Lidar and Radar as mentioned, and many more depending on the domain (for instance, a robotic arm might use force/torque sensors at the wrist to feel contact forces).
Sensors typically output either analog signals (e.g. a thermistor’s resistance changes with temperature, producing an analog voltage divider output) or digital data (e.g. a lidar might output distance readings over a serial link). For analog sensors, the microcontroller’s ADC needs sufficient resolution and sampling rate. Engineers must consider sensor accuracy, precision, noise, and response time. Often filtering techniques (like moving averages or more advanced Kalman filters) are applied to sensor readings to reduce noise. Actuators: Actuators are components that the system controls to perform physical actions. Common actuators in robotics include: electric motors of various types (DC motors, servo motors with position control, stepper motors, brushless motors for drones, etc.), linear actuators or solenoids (for pushing or lifting), relays or valves (to control power or fluid flow in industrial robots), lights or displays for signaling, and even things like speakers (for sound) or heaters (in an industrial process). In an autonomous car, actuators include the steering motor, throttle (engine or motor controller), brake actuators, and also smaller ones like mirror adjusters or indicator lights. Interfacing actuators often requires dealing with power and drivers. Microcontroller I/O pins usually cannot drive a motor or high-power device directly because of limited voltage/current. Instead, the microcontroller sends a low-power control signal to a driver circuit or module. For example, a DC motor might be controlled by an H-bridge driver (which uses transistors to switch the motor’s supply on/off or reverse polarity under microcontroller PWM control). The driver isolates and amplifies the control signal. This is why, as the robotics handbook notes, actuators are interfaced via drivers due to different power domains – the MCU might run at 5V/3.3V logic and a few milliamps, whereas a motor might need 12V and 2 amps, so a driver (transistor or MOSFET, or dedicated driver chip) is in between
erc-bpgc.github.io
. Selecting proper transistors or motor drivers, flyback diodes (for inductive loads), and understanding current draw are all part of embedded interfacing. Another consideration is feedback from actuators. Often actuators have internal sensors (e.g., servo motors have potentiometers to report position, brushless motors use encoders or back-EMF sensing). For precise control (say you want a motor to move to a specific angle), you need that feedback sensor and to implement a control algorithm (PID loop) which we’ll touch on in the next section. Prototyping and PCB Design: In early development, one might use a breadboard or perfboard to wire up microcontroller boards (like an Arduino or an STM32 Nucleo board) with sensors and actuators. This is great for experimentation. But for a robust, long-term project, designing a PCB (Printed Circuit Board) is often necessary. Tools like EAGLE or KiCad are used to draw schematics and layout PCBs
erc-bpgc.github.io
. This goes beyond pure programming – it involves electronics design (placing microcontroller, voltage regulators, connectors for sensors/actuators, etc., and ensuring signals and power are routed properly). While not every roboticist needs to be an expert in PCB design, understanding the electronics at this level is highly beneficial for debugging hardware issues and customizing your system’s design. In summary, this layer of the roadmap is about mastering how to make the hardware do things: reading sensor data correctly (and understanding its meaning/calibration) and driving actuators safely and effectively. It blends electrical engineering with software. A strong foundation here is crucial before moving on to higher-level robotics logic.
Control Systems in Robotics
Moving up the stack, we reach control systems, which deal with how to make a system behave in the desired way over time. Control theory provides the mathematical framework to regulate dynamic systems (anything that changes with time, like a motor’s speed, a robot’s position, or a drone’s altitude). At its core, a control system compares the system’s current state (as measured by sensors) to a desired state (the reference or setpoint) and tries to minimize the difference (error) by commanding actuators. This concept is known as feedback control or closed-loop control. A classic block diagram of a closed-loop controller is shown below, where the controller continuously adjusts the input to the system (plant) based on the error between desired and measured output, accounting for disturbances: 
https://www.techtarget.com/whatis/definition/closed-loop-control-system
Closed-loop control system: the controller uses feedback from sensors to adjust actuators and drive the system output toward the reference. A disturbance is any external influence that affects the system. In a robot, for example, if we want a mobile robot to maintain a speed of 1 m/s, the control loop will measure the actual speed (via wheel encoders), subtract it from 1 m/s to get an error, then adjust the motor voltage (or PWM duty cycle) to reduce that error. This runs continuously, say many times per second. Open-loop vs Closed-loop: An open-loop system sends commands to actuators without checking if the desired result was achieved (no feedback). For instance, simply running a motor at a certain power for 5 seconds to move a robot forward an approximate distance is open-loop – disturbances like friction or incline could cause the actual distance to differ. Closed-loop control adds sensing to correct for these differences automatically
techtarget.com
techtarget.com
. Virtually all advanced robotics use closed-loop control for accuracy and robustness. PID Control: The most famous feedback controller is the PID controller – Proportional, Integral, Derivative. PID mathematically combines three terms based on the error: a proportional term (P) that applies a correction proportional to the current error, an integral term (I) that corrects accumulated past errors (to eliminate steady-state offset), and a derivative term (D) that anticipates future error by reacting to the rate of change. Tuning the PID gains is an art (or a science involving frequency domain analysis) to achieve stable and responsive control. For many mechanical systems (motors, steering, balancing robots), a PID loop is sufficient to achieve good performance. For example, a drone’s flight controller uses PID loops to maintain orientation: gyroscope measures an angle error, and PID adjusts motor speeds. In an autonomous car, a PID might control steering angle to minimize lane center offset or control throttle to maintain cruise speed. The mathematical foundation here involves differential equations of motion and Laplace transforms to analyze system response (though one can often tune PID empirically). System Modeling: To design a control system, it helps to have a mathematical model of the plant (the system being controlled). This could be a set of differential equations or transfer functions describing how outputs respond to inputs. For instance, a DC motor’s speed relates to voltage input by a first-order linear equation (with motor constants for torque and back-EMF). A pendulum or two-wheeled balancing robot has nonlinear dynamics that can be linearized for control design. Control theory provides tools like modeling in state-space form, finding eigenvalues (poles of the system), and analyzing stability (e.g., via the Routh-Hurwitz criterion or Bode plots for frequency response). The advanced side of this includes designing controllers with methods like pole placement, LQR (Linear Quadratic Regulator), or using state observers (like Kalman filters) to estimate unmeasured states. From Basic to Advanced Control: Start with PID as a workhorse for many problems. When PID isn’t enough (say the system has complex multi-variable interactions or constraints), one might use more advanced methods. Feed-forward control can anticipate known system dynamics (e.g., adding a term to a motor controller to account for incline). Adaptive control can change parameters on the fly if the system behavior changes. Optimal control and Model Predictive Control (MPC) solve an optimization problem at each time step to handle constraints and future reference trajectories – for example, MPC is widely used in autonomous vehicle motion control to handle constraints like maximum acceleration and to optimize passenger comfort in trajectory following
streetdrone.com
streetdrone.com
. In robotics, control systems are everywhere: low-level motor drivers, high-level behaviors, and even in maintaining stable flight for drones or balance for bipedal robots. A good example described earlier is the feedback loop: the controller takes a reference state (desired value) and sensor feedback, and generates a control signal to drive the system toward that reference
erc-bpgc.github.io
. If the robot goes uphill (increased load), a well-tuned controller will automatically increase motor effort to maintain speed; going downhill, it will back off power or even brake to not overshoot – this adaptability is exactly why we need feedback control
erc-bpgc.github.io
. Tools for Control Design: Engineers often use tools like MATLAB/Simulink or Python libraries to simulate control systems. MATLAB has a Control Systems Toolbox where you can model a system and design/tune controllers (e.g., using root locus or bode plot methods). Simulink allows block-diagram simulation of controllers with nonlinear plants. Python has libraries (like control, simpy, or just using SciPy) that can do similar computations. There are also real-time considerations: if implementing a control loop on a microcontroller, you ensure the loop runs at a fixed frequency and in real-time (hence often using an RTOS or timer interrupts on bare-metal). In summary, mastering control systems means understanding how to maintain stability and performance of your robot’s motions under various conditions. The math behind it (differential equations, Laplace transforms, linear algebra for state-space) is fundamental, but practically one learns by applying controllers to actual hardware and iteratively tuning. With that in hand, we move to understanding the mechanics and higher-level algorithms of robots.
Kinematics and Dynamics of Robots
Robotics is multidisciplinary; aside from electronics and software, a core piece is mechanics – specifically, robot kinematics and dynamics. Kinematics is about the geometry of motion (positions, velocities, accelerations) without regard to forces, while dynamics includes forces and torques that cause motion. Degrees of Freedom (DoF): A key concept is DoF – the number of independent movements a robot or mechanism can perform. For example, a free-flying rigid body in 3D space has 6 DoF (3 translation, 3 rotation). A robotic arm’s DoF equals its number of independent joints. Understanding DoF is crucial because it tells you how maneuverable the robot is and what kind of motions it can achieve
library.fiveable.me
. For instance, a typical industrial manipulator might have 6 joints (6 DoF) to position its end-effector (like a hand) at any point with any orientation in space. Robot Kinematics: Robot kinematics applies geometry to study movements of these multi-link systems
en.wikipedia.org
. There are two main types of kinematic analysis:
Forward Kinematics: Given the joint parameters (e.g., angles of each motor in a robotic arm), compute the end-effector’s position and orientation in space. This is usually straightforward – you apply the chain of link transformations to get the result.
Inverse Kinematics: Given a desired end-effector position/orientation, compute the joint parameters needed. This is often more challenging, as it may have multiple solutions or none, and often requires solving nonlinear equations.
For a simple example, consider a 2-joint planar arm. Forward kinematics uses trigonometry to find the (x, y) of the hand given the two angles. Inverse kinematics involves solving for the angles that achieve a given (x, y). For more complex robots, one uses homogeneous transformation matrices (4x4 matrices combining rotation and translation) to represent each link’s pose. Techniques like the Denavit-Hartenberg (DH) convention are used to parameterize the chain of transformations. Kinematics is vital for planning movements. A mobile robot’s kinematics might define, say, how wheel speeds relate to the vehicle’s velocity (differential drive kinematics or car’s Ackermann steering geometry). Kinematic constraints also matter – for instance, a car is non-holonomic (it can’t move sideways due to wheel constraints) which affects path planning. Robot Dynamics: Dynamics goes a step further and involves the forces that produce accelerations (via Newton’s laws). In robotics, dynamics gives us equations of motion. For example, the Euler-Lagrange formulation or Newton-Euler algorithms can derive equations for a robot arm, taking into account masses, inertias, and external forces. Dynamics tells us things like how much torque each motor must produce to move the arm in a certain way. It also explains phenomena like momentum and inertia coupling (e.g., a heavy robotic arm segment causing oscillations). In formal terms, robot dynamics studies the relationship between the motion of a robot (velocities, accelerations) and the forces/torques that cause this motion
en.wikipedia.org
. For instance, it answers: if I want a joint to follow a certain trajectory, what torque profile is required? Or, if a sudden force hits the robot, how will it move? Dynamics is crucial for simulation (predicting how the robot will move under certain inputs) and for advanced control strategies. In feedforward control, one uses the dynamic model to compute the necessary actuator commands directly. Also, in trajectory planning, knowing the dynamic limits (like max acceleration due to motor limits) is important. Examples:
For an autonomous car, kinematics would deal with the car’s motion (bicycle model: steering angle and velocity giving a turning radius), while dynamics would include tire forces, friction, and inertia to determine how quickly it can change velocity or direction.
For a legged robot, kinematics helps compute foot placements, while dynamics is needed to handle balancing (ensuring the center of mass stays within support, computing required joint torques to not tip over).
For a robotic arm, kinematics gives the possible positions; dynamics tells you if the motors can actually lift a certain weight at a certain speed (torque = inertia * angular acceleration + gravity effects, etc.).
From a learning perspective, starting with basic kinematics (maybe even just doing geometry problems of linkages) then moving to dynamic equations is the way to go. Many robotics courses delve deeply into these, often using linear algebra (rotation matrices, Jacobians for mapping joint velocities to end-effector velocities) and calculus (for deriving motion equations). Mathematical Note: The kinematic equations often involve trigonometric functions (from rotation matrices) and can be compactly expressed using matrix exponentials or homogeneous coordinates. The Jacobian matrix of a robot arm is an important concept – it relates joint velocities to end-effector linear/angular velocity, and its transpose relates forces at the end-effector to torques at the joints. When the Jacobian is singular, the robot is at a kinematic singularity (loses a degree of freedom instantaneously). These are critical considerations in manipulator design and operation. Tools: There are libraries to help with kinematics/dynamics. For example, the Robotics Toolbox for MATLAB (by Peter Corke) or its Python equivalents can compute DH parameters and solve inverse kinematics. There are also simulation tools like Gazebo or Webots where you define a robot’s physical parameters and let the physics engine handle the dynamics, which is great for testing controllers and algorithms. Understanding kinematics and dynamics bridges the gap between a robot’s physical design and the software that controls it. It informs how you design your control laws and plan trajectories that the robot can actually follow. With these foundations, we can now talk about the higher-level software architecture that ties everything together, particularly using the Robot Operating System.
Robot Software Architecture and ROS
As robots became more complex, the need for a modular software framework became apparent. You wouldn’t want to write one giant monolithic program to control a whole robot – that would be unmanageable. Instead, robot software is structured into many modules (or processes) that handle different aspects (locomotion, perception, planning, etc.) and communicate with each other. The de facto standard framework enabling this is ROS – Robot Operating System. Despite its name, ROS is not an actual OS, but a middleware layer on top of an operating system (usually Linux) that provides services like message passing, package management, and device drivers in a unified way
erc-bpgc.github.io
. ROS allows you to split robot functionality into nodes (each node is like a small program). For example, one node might read sensor data, another does SLAM, another handles motion planning, another controls actuators. These nodes can run concurrently and even distributed across multiple computers. ROS handles the communication between them through a publish/subscribe model of topics (for streaming messages like sensor data) and services/actions (for request-response or long-running goals). The benefit of ROS is that it provides standardization. There are standardized message types (e.g., geometry_msgs/Twist for velocity commands, sensor_msgs/LaserScan for lidar data, etc.) and many community-contributed packages. If you need a driver for a LiDAR or camera, chances are someone wrote a ROS driver already. Need to perform path planning? ROS has the MoveIt package for motion planning or navigation stack for mobile robot path planning, which can be configured rather than written from scratch. ROS 1 vs ROS 2: ROS has evolved – ROS 1 (up to ROS Melodic/Noetic) was widely used in research and prototyping, but it had limitations (not real-time safe, central master node, mainly C++ and Python). ROS 2 (newer) was designed for industry and embedded use: it uses DDS (Data Distribution Service) for peer-to-peer communication, is better for real-time and reliability, and has security features. Many autonomous vehicle companies use ROS 2 as part of their stack (for instance, Autoware.auto is an autonomous driving software built on ROS 2). Software Architecture Pattern (Sense-Plan-Act): A common design pattern in autonomous systems is the Sense → Plan → Act loop
streetdrone.com
streetdrone.com
. Sensing modules perceive the environment, planning modules decide what to do, and actuation modules carry it out. ROS facilitates implementing this by separating those concerns. The hierarchical robotic paradigm could be represented as: 
https://www.streetdrone.com/hello-world-sense-plan-act/
The classic “Sense-Plan-Act” architecture in robotics: sensor data is processed (Sense), decisions are made (Plan), and commands are executed (Act), in a continuous loop. In ROS terms:
Sensing would be handled by Perception nodes: e.g., processing camera images, LiDAR scans, IMU data to produce meaningful info (like obstacle positions, a map of the environment, or the robot’s localized position).
Planning is done by Planning/Decision nodes: e.g., given a goal, plan a path or decide the next movement; this could involve global path planning, local obstacle avoidance, or high-level behavior selection.
Acting is done by Control/Actuation nodes: e.g., taking a planned trajectory and sending low-level commands to motors using control algorithms (like sending wheel velocities, steering angles, or joint torques).
These nodes communicate via topics like “/camera/image_raw” for raw camera feed, “/cmd_vel” for a commanded velocity, “/odom” for odometry feedback, etc., which are standard in ROS ecosystem. A concrete example: In a mobile robot using ROS, you might have a LiDAR publishing laser scans. A SLAM node subscribes to those scans and “/cmd_vel” (wheel odometry) and publishes the robot’s estimated pose on “/odom” and a map on “/map”. A navigation node takes the map and pose and a given goal, then publishes a planned path or velocity commands to “/cmd_vel”. Finally, a motor driver node subscribes to “/cmd_vel” and interfaces with the hardware to set motor speeds. This decoupling means you can swap out implementations (maybe use a different SLAM algorithm) without changing the rest, as long as the interfaces remain consistent. Simulation and Visualization: ROS also offers tools like Gazebo, a 3D physics simulator where you can spawn robot models in virtual environments and test algorithms without physical hardware. Gazebo can simulate sensors (cameras, LiDAR, IMU, etc.) and robot dynamics reasonably well. We also have RViz, a powerful visualization tool to see sensor data and robot state (great for debugging perception and planning algorithms in real-time). These tools accelerate development and are heavily used in both research and industry prototyping. Other Frameworks: While ROS is prevalent, there are other frameworks and architectures (especially in specific industries). For instance, some industrial robots use proprietary real-time frameworks or PLCs (programmable logic controllers) for control. But ROS has even penetrated those areas as a higher-level coordination layer. In academia and open-source, ROS remains the go-to. There are also middleware like LCM (Lightweight Communications and Marshalling) from MIT, or YARP in some robotics labs, but they are less common than ROS today. Languages: ROS is primarily used with C++ (for performance-critical nodes) and Python (for ease of scripting), though it supports other languages. Most performance-heavy libraries (for vision, planning, etc.) are in C++ for speed. Python is often used for quick integration and experiments. ROS 2 introduced more robust support for multiple languages and real-time, but C++ and Python are still dominant. Common Robotics Libraries: Within the ROS ecosystem or alongside it, there are many libraries researchers and developers use:
OpenCV for image processing (often integrated in ROS nodes for camera data)
erc-bpgc.github.io
erc-bpgc.github.io
.
PCL (Point Cloud Library) for 3D point cloud processing (used with 3D sensor data like LiDAR or depth cameras)
erc-bpgc.github.io
erc-bpgc.github.io
.
MoveIt for motion planning of robotic arms (computing collision-free trajectories in configuration space).
Navigation Stack for mobile robots (uses algorithms like Dijkstra/A*, costmaps for obstacle avoidance, etc., behind a relatively easy interface).
TF (transform library) for keeping track of multiple coordinate frames (essential in robotics to know the pose of sensors relative to robot base, etc., at all times).
Gazebo/Ignition for simulation as mentioned.
Robot-specific drivers: e.g., if using a drone, the PX4 autopilot software can interface with ROS; for a robotic arm, the manufacturer might provide a ROS driver package.
To summarize, ROS and similar frameworks enable the modularity and scalability needed in complex robotics software. They encourage reusing and sharing code (since many robots need the same core functions like SLAM or path planning). As you progress, learning ROS is practically a must for robotics researchers – it also teaches good practices in distributed system design and software integration. With the software framework in place, we now move on to the high-level capabilities: perception, localization, planning, and so on, which run on top of this architecture.
Perception and Computer Vision in Robotics
Perception is how robots make sense of raw sensor data to understand their environment. This is arguably one of the most challenging aspects of robotics and autonomous systems – tasks that are trivial for human perception can be extraordinarily complex for a robot
erc-bpgc.github.io
. For example, recognizing a face, identifying an obstacle, or understanding a spoken instruction are perception problems that involve sensing and interpretation. In the context of autonomous driving and robotics, perception typically includes:
Processing camera images (Computer Vision): to detect and classify objects (pedestrians, other vehicles, traffic signs in driving; or for a home robot, recognizing a door or a person, etc.), to estimate depth (via stereo vision or structure-from-motion), or to track movements. Classical computer vision methods involve techniques like edge detection, optical flow, feature extraction (SIFT, ORB features for instance) and geometric computation for things like stereo depth maps
erc-bpgc.github.io
. Modern approaches rely on Convolutional Neural Networks (CNNs) to do tasks like image classification, object detection (e.g., YOLO, Faster R-CNN algorithms), and semantic segmentation (labeling each pixel by object class)
erc-bpgc.github.io
. These have seen huge success – e.g., CNN-based systems can outperform humans in some image recognition tasks.
Lidar and 3D perception: processing point clouds to detect obstacles, categorize terrain, or build 3D maps. Techniques include clustering of points to find objects, plane fitting (e.g., to detect the ground plane), and using 3D voxel grids or OctoMaps for environment representation. There are also learning-based methods now for point clouds (like PointNet). The Point Cloud Library (PCL) provides many algorithms for filtering and extracting features from point cloud data
erc-bpgc.github.io
erc-bpgc.github.io
.
Sensor Fusion: combining data from multiple sensors to get a better estimate than any single sensor alone. For instance, a camera might fail in low light, but a radar still works – fusing them yields robust obstacle detection day or night
blackberry.qnx.com
blackberry.qnx.com
. Sensor fusion often relies on probabilistic approaches (like Kalman filters or particle filters) to integrate information over time and from different sources. In an autonomous car, a classic fusion problem is combining Lidar, camera, and radar to reliably perceive a pedestrian and know their distance and velocity. The Kalman Filter (and its non-linear variants like Extended or Unscented Kalman Filters) is a widely used mathematical tool here, as it provides an optimal recursive estimator under certain assumptions.
State Estimation: determining the robot’s own state from sensor data. For example, using an IMU, wheel encoders, and maybe GPS to estimate a robot’s pose (position and orientation) over time is a perception problem (often overlapping with localization, see next section). This could be considered part of perception because it’s interpreting sensor data to get meaningful state info (like velocity, tilt angle, etc.).
A fundamental concept is the depth map – for stereo vision, two cameras can produce a disparity map which correlates to depth (our two eyes do the same)
erc-bpgc.github.io
. Depth maps or point clouds let the robot perceive the 3D structure of the scene, which is essential for tasks like navigation or manipulation. For example, a robot vacuum needs to sense obstacles in 3D to avoid bumping into furniture. Perception Challenges: Robots often struggle with perception due to noise, partial observability, and computational constraints. For instance, lighting changes or motion blur can screw up camera algorithms; LiDAR gives precise range but might not detect glass or certain surfaces well; each sensor has strengths and weaknesses
blackberry.qnx.com
blackberry.qnx.com
. That’s why multiple sensors are used together. The understanding part is hard: identifying that an object in camera view is a pedestrian vs just a pattern of pixels is what CNN-based deep learning has helped significantly with in the last decade
erc-bpgc.github.io
. Similarly, segmenting an image (which pixels belong to road vs. sidewalk vs. cars) is vital for an autonomous car to navigate safely. Machine Learning in Perception: This will be touched more in the AI section, but it’s worth noting here that most modern perception systems use machine learning models. For instance, neural networks trained on large datasets (like ImageNet for generic objects, or KITTI/Cityscapes for driving scenes) can detect and classify objects in real-time. LiDAR point cloud processing also uses ML, e.g., networks that take a 3D point cloud and output locations of other vehicles and pedestrians. These are computationally heavy but with specialized hardware (GPUs, TPUs) they can often run on-board an autonomous vehicle. Example – Autonomous Driving Perception: In a self-driving car, perception might involve:
A Vision module that uses cameras to detect traffic lights color, recognize road signs, identify drivable lanes (lane detection), and see objects. Deep learning models (like segmentation nets or detection nets) handle this.
A LiDAR module that builds a 3D occupancy grid or point cloud of the surroundings and clusters points to find obstacles.
A Radar module that directly measures speeds of objects and complements LiDAR for longer range and adverse weather.
Then a sensor fusion module that takes all those detections and creates a unified “scene understanding”, often in the form of a list of tracked objects (with position and velocity) and maybe a grid map marking free vs occupied space.
Perception in Robotics (non-vehicles): For a robot arm, perception might be using a camera to locate an object to pick up (the field of computer vision overlaps with robotics heavily here, including things like feature matching for object recognition). For a humanoid robot, perception might include auditory (speech recognition) and tactile sensing as well. In terms of mathematics, perception algorithms use a lot of linear algebra (images are matrices of pixels; operations like convolution are essentially matrix operations), probability and statistics (sensor noise models, Bayesian filters for fusion), and increasingly, the math of deep learning (which is still linear algebra and calculus under the hood, for backpropagation and optimizing network weights). Key Tools & Libraries:
OpenCV (Open Computer Vision library) – a powerful toolkit for image processing, containing algorithms for filtering, feature detection, camera calibration, etc., widely used for classical vision tasks
erc-bpgc.github.io
.
PCL (Point Cloud Library) – as mentioned, for 3D point cloud processing
erc-bpgc.github.io
.
Open3D – a newer library for 3D data processing and visualization.
Deep learning frameworks like TensorFlow or PyTorch for implementing and running neural networks that perform perception tasks
erc-bpgc.github.io
.
YOLO, Detectron2, etc. – specialized object detection frameworks.
ROS also has packages like vision_opencv (bindings for OpenCV in ROS), image_pipeline, and various detectors.
Perception is often the rate-limiting factor in autonomy: you cannot make decisions or control effectively if your robot is “blind” or misinterpreting the environment. It’s an area of intense research – e.g., improving how AI handles edge cases (a stop sign partially obscured by graffiti, or unusual weather conditions). As a practitioner, building a perception system involves choosing the right sensors and ensuring your algorithms are robust for the scenarios the robot will encounter. Now, once a robot can perceive its environment, it also needs to localize itself and build maps, which leads us to SLAM.
Localization and Mapping (SLAM)
For a robot to navigate, it must know where it is in the environment (localization) and often have a map of the environment. These problems are deeply connected: a robot uses its sensors to build a map while simultaneously figuring out its own location within that map – this is the classic SLAM problem: Simultaneous Localization and Mapping. In earlier days, mapping and localization were tackled separately. For instance, if you already have a detailed map (say a floorplan of a building or a pre-scanned 3D map of city streets), the task reduces to localization: using sensors (camera, LiDAR, GPS, etc.) to recognize features and pinpoint the robot’s pose on that known map. Conversely, if you only care about mapping (like surveying a cave with a drone), you might not worry about precise localization beyond stitching sensor data into a consistent map. SLAM does both together, which is hard because you don’t know the map a priori and your position is uncertain – and each uncertainty feeds into the other
erc-bpgc.github.io
. The SLAM problem can be stated like: as the robot moves, it takes sensor measurements (like distances to walls, or visual landmarks) from various unknown poses. We want to estimate both the robot’s trajectory and the map of landmarks or occupancy grid that is consistent with all those measurements. Mathematical foundation: SLAM is often solved using probability and linear algebra. The robot’s pose and map can be seen as variables in a large optimization problem. Early approaches used Extended Kalman Filters (EKF): the state vector contains the robot pose and positions of landmarks, and each new sensor measurement updates this state probabilistically. EKF-SLAM works but can get computationally heavy as map size grows (state vector large). Another approach is Particle Filter based SLAM (like FastSLAM algorithm), which uses particles to represent possible robot poses and builds maps for each – this scales better with landmarks. Modern SLAM often uses graph optimization: every robot pose and landmark is a node in a graph, and sensor measurements add constraints (edges) between these nodes (e.g., “range from pose_i to landmark_j was 5m”). Then the whole graph is optimized (e.g., via least-squares solvers) to best fit all measurements – this is called Graph-SLAM or bundle adjustment in vision contexts. A famous example in 2D is the GMapping algorithm (particle filter based) and Cartographer (graph-based by Google). In visual SLAM (using cameras), algorithms like ORB-SLAM use visual features as landmarks. Localization (when a map is given) often uses Monte Carlo Localization (MCL), which is a particle filter: you have many guess poses (particles) and as sensor data comes in, you weight and resample them to converge on the correct pose. This is used in ROS’s standard navigation stack to localize a robot on a known floorplan using LiDAR. Mapping representation can be feature-based (a list of landmark points) or occupancy grid (dividing space into cells marked free/occupied). Occupancy grids are common for 2D mapping with Lidar – basically producing a bitmap of the environment. For 3D, one might use occupancy voxels or point clouds. Use of SLAM: If a robot is dropped in an unknown environment, SLAM lets it explore and chart out the space while knowing where it is within that space. Think of a Mars rover – it lands with maybe no detailed map of the terrain; as it drives, it builds up a map of rocks and landmarks, and always keeps track of its position relative to where it landed. In autonomous driving, pure SLAM (like to build a map of an entire city) is not usually done by the car on the fly – instead, companies pre-map areas with lidar and HD cameras. The car then does localization against those HD maps (this is how many systems achieve lane-level accuracy using vision/lidar to match known map features). However, some scenarios require online SLAM: e.g., a low-speed shuttle in a constantly changing environment might update its map frequently. Also, loop closure is a SLAM concept: when the robot revisits a place, recognizing it can greatly reduce drift in the map. For example, a self-driving car coming back to a previously seen intersection can correct any accumulated pose error by aligning its map to the known landmarks of that intersection – this is part of graph optimization in SLAM. Challenges: SLAM algorithms need to handle sensor noise and potential data association problems (mistaking one landmark for another). Computationally, it can be intensive, but modern methods and computing power allow real-time SLAM for moderately large environments. Robust SLAM in dynamic environments (with moving objects) is still an active research area. Linear Algebra and Probability: As hinted, linear algebra appears in forming Jacobians for EKF or in solving the normal equations for graph optimization (solving large sparse matrices for the best fit map). Probability is at the heart of state estimation – concepts like Bayes’ rule, Markov processes, etc., are heavily used. It’s recommended to understand at least basic probability and matrix operations to grasp SLAM deeply. Tools: Many SLAM implementations are open-source. For example, ROS has Gmapping, Cartographer, HectorSLAM, Karto, etc. For visual SLAM, there are libraries like ORB-SLAM2, RTAB-Map (which does RGB-D SLAM combining vision and depth sensors), etc. There are also datasets like KITTI, EuRoC MAV, TUM RGB-D that are used to benchmark SLAM algorithms. SLAM is a beautiful synergy of perception and state estimation. Once the robot can build a map and localize itself, the next question is: how to plan motions in that map to achieve goals? That is the realm of planning.
Path Planning and Motion Planning
Path planning is the process of finding a feasible route or trajectory from point A to point B, typically while avoiding obstacles and respecting the robot’s kinematic or dynamic constraints
erc-bpgc.github.io
. In essence, given a map of the environment and a desired goal location (or configuration), the planner figures out how to get there. This is analogous to how a GPS navigation system finds a route on roads for your car, but in robotics it can be more complex because the “roads” may not be predefined and the robot’s movement constraints vary. For mobile robots (like a wheeled robot or autonomous car), path planning often happens in two layers:
Global Planning: Plans a coarse route to the goal, often on a graph representation of the environment (e.g., grid or roadmap). If you have a known map (grid map), algorithms like Dijkstra or A* (A-star) can be used to find the shortest path on that grid from start to goal
erc-bpgc.github.io
. These are graph-based algorithms that guarantee finding an optimal path (shortest, or lowest cost given a costmap) if one exists. In large outdoor maps, you might use variants like D* or Theta*.
Local Planning (Obstacle Avoidance): Reacts to obstacles (especially moving ones or those not in the global map) and ensures smooth, feasible motion. Techniques include Dynamic Window Approach or Velocity Obstacles for robots to avoid collisions in real-time. This often ties into control – generating velocity commands that steer the robot around obstacles.
In robotics, path typically refers to the geometric route, whereas trajectory usually implies path + timing (i.e., how to move along the path as a function of time). Motion planning encompasses both – often implying we consider the full state and possibly dynamics. For a robotic manipulator arm, motion planning involves figuring out joint angle paths that move the end-effector from one pose to another without hitting anything. For an autonomous car, motion planning might involve computing a sequence of steering and speed commands that follow a lane and avoid other vehicles smoothly. Algorithms:
Graph-Based Planning: As mentioned, grid or graph search with A*. You discretize the space (or use waypoints) and search. A* uses a heuristic (like straight-line distance) to guide the search efficiently. Variants like ARA (Anytime Repairing A*) can trade off optimality for speed, or Hybrid A** which plans in a continuous-curvature space for cars.
Sampling-Based Planning: In high-dimensional spaces (like a 6-DOF robotic arm’s joint space), grid search becomes impractical. Sampling-based planners like RRT (Rapidly-Exploring Random Tree) or PRM (Probabilistic Roadmap) are used
erc-bpgc.github.io
. RRT quickly searches the space by random sampling and grows a tree towards the goal, and can handle complex spaces. PRM pre-samples the space to build a roadmap graph. These algorithms are probabilistically complete (they find a solution if one exists, given enough time).
Optimization-Based Planning: These treat planning as an optimization problem: define a cost function (like path length, or energy, or closeness to obstacles) and constraints (dynamics, avoiding collisions) and use techniques like gradient descent, Sequential Quadratic Programming, or modern trajectory optimization (e.g., CHOMP or TrajOpt for manipulators). Model Predictive Control (MPC) can be seen as a type of on-line optimization-based planning, where at each time step it solves an optimization for the next few seconds of motion.
Behavior Planning: In driving, there's a concept of high-level behavior or decision making (e.g., decide to change lane or yield). This can be seen as above planning – often done with state machines or decision networks that then invoke path planners.
Kinodynamic Planning: Planning that accounts for dynamics (kinematic + dynamic constraints). For example, a quadcopter can’t stop or turn instantaneously; a kinodynamic planner ensures the path found respects acceleration limits and such. RRT* (an optimized version of RRT) and other variants exist for kinodynamic cases.
The text above mentions that path planning is integral because robots “find it difficult” to plan by themselves; we must provide algorithms to imbue them with this capability
erc-bpgc.github.io
. Even moving a robotic arm from one point to another in free space involves planning a collision-free trajectory (even if straight-line in joint space might work, you have to check it doesn’t hit an object or itself). The quote also notes that even robotic arms require trajectory planning for the end-effector
erc-bpgc.github.io
 – indeed, motion planning isn’t just for mobile robots. Prediction and Uncertainty: A more advanced aspect, especially in autonomous driving, is planning under uncertainty and predicting other agents. For instance, an autonomous car must predict the future positions of nearby vehicles and pedestrians (this is often treated as a separate module, prediction, using methods like Kalman filters or recurrent neural networks on agent behavior)
medium.com
. The planner then considers these predictions to avoid future collisions – not just current positions. This merges into the domain of decision making and game theory (when other agents are decision-makers too). Mathematical aspects: Graph search algorithms rely on algorithmic complexity analysis and heuristic admissibility proofs (A*). Sampling algorithms might involve probability theory for their guarantees. Optimization-based methods obviously use calculus (Jacobians of constraints, etc.) and numerical optimization techniques. Available Tools:
ROS’s Navfn or SBPL packages implement A* and other planners for mobile bases. The ROS Navigation Stack uses a global Dijkstra/A* planner and a local planner (like DWA).
OMPL (Open Motion Planning Library) is a great C++ library with many planning algorithms (RRT, PRM, etc.) that can be used for robot arms or other planning problems
erc-bpgc.github.io
. It’s integrated into ROS MoveIt for planning arm motions, for example. (OMPL is C++ heavy as noted
erc-bpgc.github.io
, but it abstracts a lot).
Higher-level frameworks like MoveIt provide a lot of functionality out-of-the-box: you give it a robot’s URDF model and a goal, and it will attempt to plan a joint-space path to reach the goal, using any of the available algorithms internally.
Path Planning Visualization: It helps to visualize planning algorithms to understand them. There are educational tools and GitHub repos (like the one referenced in the roadmap
erc-bpgc.github.io
) that show how algorithms like A*, RRT, etc., explore the space and find paths. This is useful to build intuition. To tie it up: path planning ensures the robot can chart a safe and efficient course of action to achieve its objectives. Once a path or trajectory is decided, it’s handed over to the control subsystem to execute. Finally, we integrate Machine Learning and AI concepts which increasingly permeate all levels of this stack, particularly in perception and decision-making.
Machine Learning and AI in Robotics
In recent years, AI and Machine Learning (ML) have become central to robotics and autonomous systems, especially for perception and decision-making tasks that are hard to solve with manual rules. The key difference between a traditional programmed system and an AI-based system is that the latter learns from data/examples rather than being explicitly coded for every scenario
erc-bpgc.github.io
. For example, rather than hand-coding a routine to detect pedestrians in camera images (which would be almost impossible to cover all appearances), we train a deep neural network on thousands of labeled images of pedestrians and let it learn the features that identify a pedestrian. This learning-based approach has proven extremely powerful. Where AI is used:
Computer Vision and Perception: as discussed, CNNs for image recognition, segmentation, etc. Also, Deep Reinforcement Learning can be used for end-to-end perception-action mappings (like learning to drive by raw pixels, though that’s more experimental).
Localization/Mapping: ML can enhance SLAM (loop closure detection via vision is often done with CNN features now). Some research uses neural nets to predict place recognition (kidnapped robot problem).
Control and Planning: While classical control still rules low-level, there are instances of ML like reinforcement learning training a policy (e.g., training a quadruped robot to walk via trial and error simulations). Model-predictive controllers can integrate learned models of vehicle dynamics. Also, imitation learning might be used for planning – an autonomous car policy network could be trained from human driving data to mimic driving behaviors (with caution, of course).
High-Level Decision Making: For instance, behavior planning (like deciding lane changes) could use ML to evaluate risk or mimic human decisions. And in multi-agent scenarios, game-theoretic or RL approaches might be applied to negotiate merges or intersections.
Robotics Learning: there’s a whole field of robot learning focusing on things like learning from demonstration, where a human demonstrates a task and the robot learns the policy (this involves algorithms like dynamic movement primitives, Gaussian mixture models, etc.).
Neural Networks and Deep Learning: These are the backbone of modern robotics AI. Convolutional Neural Networks (CNNs) handle spatial data (images, 3D voxels). Recurrent Neural Networks (RNNs, or more commonly now LSTMs, GRUs, Transformers) handle sequential data and time series (perhaps prediction of trajectories). For example, a network might take as input a sequence of a car’s positions and predict its next position (useful for predicting another vehicle’s future path). Deep learning models require a lot of data and compute to train – often this is done offline (e.g., training an object detector on a server farm with GPUs). The resulting model (a bunch of learned weights) is then deployed on the robot. Deploying may require optimization (like quantization or using accelerators) to run in real-time on embedded hardware. Reinforcement Learning (RL): RL is training an agent to take actions in an environment to maximize cumulative reward. In robotics, RL can be used to learn complex behaviors that are hard to script, like legged locomotion or acrobatic drone flight. With simulators, one can train policies in simulation (like using OpenAI Gym environments, or custom Gazebo environments) and then transfer to real robots (with techniques to handle the reality gap, like domain randomization). Notable successes include OpenAI’s Rubik’s cube solving robot hand (using RL + domain randomization) and Boston Dynamics-like locomotion controllers learned via RL. Mathematics of ML: It involves linear algebra (tensors, matrix multiplications in neural nets), calculus (the training process is basically solving an optimization via gradient descent – computing gradients of a loss function w.r.t. millions of weights), and probability (especially in probabilistic models or when dealing with uncertainties and distributions). A strong math foundation helps in understanding why certain networks perform better or how to tune them. Tools & Frameworks for AI:
PyTorch and TensorFlow are the dominant deep learning frameworks
erc-bpgc.github.io
. They allow you to define neural network architectures and train them on GPUs with relative ease.
scikit-learn is a useful Python library for classical ML algorithms (decision trees, SVMs, clustering, etc.)
erc-bpgc.github.io
 – sometimes simpler models are enough or can be used for quick prototyping.
NumPy and Matplotlib are staples for any scientific computing in Python (data manipulation and visualization)
erc-bpgc.github.io
.
OpenAI Gym or ROS RL libraries (like ros_openai package) can be used for reinforcement learning tasks and simulations.
There are also specialized tools like TensorRT for optimizing neural network inference on embedded platforms (NVIDIA’s Jetson platform, commonly used in robots and AVs, benefits from this).
It’s also worth noting ethics and safety in AI – for autonomous driving, using AI raises concerns about explainability and reliability. So while ML is used, it’s often combined with rule-based systems or extensively tested to ensure safety (e.g., a vision system may detect a pedestrian, but there may be a rule that if an object is detected and uncertain, the car should still brake). Example: Consider an autonomous car’s end-to-end learning approach – a neural network takes camera input and directly outputs steering angle. This was demonstrated in research (NVIDIA’s DAVE-2 system) but most industry approaches break the problem down (perception via CNN, then planning with perhaps more rule-based or optimization-based methods). The end-to-end is pure learning but harder to validate. A hybrid approach is more common: ML for perception, maybe ML for some driving policy suggestions, but constrained by known safety rules. In robotics research, you'll encounter terms like SLAM (we covered) enhanced by deep learning (for loop closure detection using CNN features), visual servoing (using vision in control loops, sometimes augmented with learning), and human-robot interaction (where natural language processing and learning might come in to interpret human commands or gestures). Machine learning is a vast topic; the key takeaway is that it provides robots the ability to improve performance by example and to handle complexity that is infeasible to hard-code. However, it usually requires plenty of data and careful validation. For someone following this roadmap: after getting comfortable with the basics of robotics, diving into courses on machine learning and deep learning (like Andrew Ng’s ML course
erc-bpgc.github.io
 or Stanford’s CS231n for vision, CS229 for ML, CS234 for RL, etc.) is highly recommended to complement robotics knowledge with AI skills.
Autonomous Driving Systems
Bringing it all together, we arrive at autonomous driving, which can be seen as a specialized (and very advanced) application of robotics. An autonomous vehicle (AV) is essentially a robot car that must operate in the dynamic and complex environment of roads among humans. It integrates everything discussed: embedded systems for low-level control, a network of sensors for perception, control systems for vehicle dynamics, planning for navigation and collision avoidance, and AI for understanding scenes and making decisions. Levels of Autonomy: The SAE defines levels 0 through 5 for driving automation
blackberry.qnx.com
:
Level 0: No automation (human drives entirely).
Level 1: Driver assistance (e.g. adaptive cruise control or lane keeping assist – one function automated).
Level 2: Partial automation (e.g. Tesla Autopilot, combining steering and throttle in some scenarios, but driver must monitor at all times).
Level 3: Conditional automation – the car can handle driving in certain conditions, but the human driver must be ready to take over if asked.
Level 4: High automation – the car can drive by itself in defined conditions or domains (like within a geo-fenced city area or highway), and does not expect human intervention in those conditions.
Level 5: Full automation – the car can handle all driving tasks in all environments a human could, with no driver needed, ever.
Most systems as of 2025 are around Level 2 (some Level 3 in testing, like traffic jam chauffeur systems)
blackberry.qnx.com
blackberry.qnx.com
. Level 5 remains extremely challenging and not yet achieved for general public use. Architecture of an Autonomous Vehicle: A typical AV has a complex architecture, often broken into subsystems similar to the sense-plan-act structure:
Perception Subsystem: Involves processing from multiple sensors – cameras, LiDARs, radars, ultrasonic, GPS, IMU, etc. – to output an understanding of the environment. This includes detecting vehicles, pedestrians, free space, traffic signals, etc., and also estimating the ego-vehicle’s precise location. In a “Waymo-like” stack, this is multi-layered with object detection and tracking
medium.com
medium.com
. The sensor fusion aspect is critical: for example, combining radar and camera helps get both the reliability of radar in weather and the detail of camera vision
blackberry.qnx.com
.
Mapping and Localization: Many AVs use HD maps (high-definition maps) which include details like lane markings, curb locations, stop line positions, etc. The vehicle localizes itself in this map using sensor data (e.g., matching Lidar scans to a map of Lidar reflectivity). This gives very precise positioning (centimeter-level). The map provides context (like “upcoming intersection has 2 lanes that turn left”). Some approaches try to minimize reliance on pre-mapped info (Tesla famously avoids LiDAR and detailed HD maps, relying more on cameras and generalized vision).
Prediction: Unique to autonomous systems that deal with other agents – the AV must predict the intentions and future trajectories of nearby cars, bikes, pedestrians. This is often handled by a prediction module that takes the tracked objects from perception and estimates where they will be in a few seconds (e.g., that pedestrian is likely to cross the street, or that car will continue straight at current speed). These predictions usually incorporate models of behavior or learned patterns.
Planning and Decision Subsystem: Based on the perceived environment, static map, and predictions, the planning subsystem decides how to get to the goal safely and comfortably. It typically has layers:
Route Planning: at the road network level (like GPS routing, which road segments to take).
Behavior Planning: deciding maneuvers like lane changes, yielding, overtaking, stopping at traffic lights, etc. This can be rule-based or ML-assisted, often represented as finite state machines or decision trees (e.g., state = “following lane”, event = slow car ahead -> transition to state “prepare lane change”).
Trajectory Planning (Motion Planning): computing the actual trajectory (steering and speed profile) for the next few seconds. This trajectory must obey vehicle kinematics (can’t turn too sharply beyond capabilities), avoid obstacles, and achieve the behavior goals (like change lane smoothly). It often involves optimization to minimize acceleration jolts and adhere to constraints. For instance, a polynomial or spline might be fitted for a lane change trajectory.
streetdrone.com
streetdrone.com
 describes this layering: route, behavioral, trajectory planning in an AV.
Control Subsystem: Takes the planned trajectory (a sequence of target positions or velocities) and commands the vehicle’s actuators (throttle, brake, steering) to follow it
streetdrone.com
. This involves low-level control loops like steering angle control (perhaps via PID) and speed control. As mentioned, PID controllers are often used for lateral and longitudinal control to minimize tracking error
streetdrone.com
. Sometimes more advanced control like MPC is used, especially if needing to handle constraints (like ensuring passenger comfort by limiting jerk, or coordinating steering and acceleration)
streetdrone.com
. The car also has to interface with drive-by-wire systems (for example, sending a CAN bus command to set steering torque). Redundancy is important: many AVs have redundant control paths or fail-safes (like if main computer fails, an emergency controller can slow the vehicle).
System Management and Safety: On top of all this, there are subsystems for monitoring health (diagnostics), cybersecurity (to prevent hacks), and fallback planning (safe stop if something goes wrong). There are also human-machine interface considerations (if there is a safety driver or passengers to alert).
In an AV software stack, these pieces often run on multiple computing units. A modern AV might have an array of computers: some GPUs for heavy neural network processing (perception), an FPGA or dedicated accelerator for sensor fusion at high rate, a CPU for planning tasks, etc. They also typically run a real-time OS or hypervisor: for instance, QNX might run the low-level control and perhaps certain perception threads, while Linux runs higher-level processes. Ensuring real-time performance (the control loop maybe needs 50 Hz reliably) and safety isolation (a crash in a UI process shouldn’t take down the braking controller) is crucial. The automotive functional safety standard, ISO 26262, classifies systems by ASIL (Automotive Safety Integrity Level), and something like the braking controller would be ASIL-D (highest), requiring rigorous validation and probably implementation on a certified RTOS, whereas the infotainment can be QM (quality managed, no specific safety requirement). Vehicles vs. Other Robots: Autonomous cars have to deal with traffic rules, high speeds, and mixed traffic with humans. This adds layers of complexity in prediction and social compliance. However, the structured environment (roads) gives some advantages (clear lanes, maps). Contrast with domestic robots that operate in unstructured home environments – they face different challenges (like random objects on the floor, pets, unknown layouts, etc.). But many fundamentals overlap. V&V (Verification and Validation): Due to safety criticality, enormous effort goes into testing autonomous systems. Simulation is heavily used (driving simulators like CARLA or custom frameworks) to test scenarios that are rare in real life (like a child running into the road) or to validate changes. Companies also do controlled track testing and gradually move to public road testing. Regulations require proving that the system is at least as safe as a human driver, which is a high bar. This is more procedural, but good to be aware of as part of the development cycle. Public and Ethical Considerations: Deploying autonomous robots (cars, drones, etc.) brings up questions of liability, ethics (decision in unavoidable crash scenarios), and public acceptance. Engineers need to work with ethicists and policymakers on these aspects, though that’s beyond the technical scope of this guide. Modern Status: As of 2025, autonomous driving technology is in an interesting phase – there are limited deployments of robotaxis in some cities (by Waymo, Cruise, etc. at Level 4 in geofenced areas), and many new cars have advanced driver-assist features (Level 2). The ultimate Level 5 goal is still some years away. The field continues to evolve with improvements in AI, better sensors (imaging radars, solid-state lidars), and more computing power. From an engineering learning perspective, working on autonomous vehicles is like a “capstone” of robotics – you need knowledge of embedded systems (for interfacing with car hardware), robust control (vehicle dynamics), advanced perception (multi-sensor fusion, 3D computer vision), planning under uncertainty, and system integration – all at once. It’s extremely challenging but also rewarding as it can greatly impact transportation and safety. 
https://www.streetdrone.com/hello-world-sense-plan-act/
High-level architecture of an autonomous vehicle software stack. Sensors (camera, radar, lidar, GPS/IMU) feed into a Perception module (with detection and localization). The Planning module performs route planning, behavioral decision-making, and trajectory planning. The Control module (vehicle interface) executes the trajectory with feedback controllers (PID, MPC) to command throttle, brake, and steering (Drive-By-Wire actuators). The above diagram illustrates how everything ties together in an AV: raw data flows from sensors into perception (which outputs an environmental model and the vehicle’s localized position). Planning takes that and produces a drive plan. Control then actuates the vehicle accordingly. Each block internally might use methods we discussed (detection uses AI vision, localization might use SLAM techniques, planning uses path planning algorithms, control uses PID loops).
Conclusion and Further Learning
Building an autonomous robotic system from the ground up is a step-by-step journey. The roadmap we’ve outlined can be summarized in progressive milestones:
Embedded Systems & C Programming: Start by learning to program microcontrollers (e.g. Arduino or STM32). Get comfortable with C/C++, digital/analog I/O, and writing simple control loops. Build small projects – for example, a line-following robot or a sensor logger. This gives insight into hardware constraints and low-level debugging. It’s the “bare metal” foundation.
Real-Time and OS Concepts: Move to using an RTOS or embedded Linux on platforms like Raspberry Pi or NVIDIA Jetson. Understand how to manage multiple tasks and real-time scheduling. Try a project where one thread reads sensors while another controls motors with a timing deadline.
Sensors and Actuators Integration: Work with a variety of sensors (ultrasonic rangefinder, IMU, camera) and actuators (DC motors with encoder feedback, servo motors). Learn how to interface them (ADC reading, PWM generation, communication protocols like I2C/SPI/CAN). Calibrate sensors and tune actuators using simple controllers.
Control Theory Application: Implement a PID controller on a real system – for instance, balance an inverted pendulum (classic cart-pole or a self-balancing two-wheel robot) or control a drone’s roll angle. This will require modeling (or trial-and-error tuning) and gives a feel for dynamics and stability concerns.
Robotics Kinematics & Dynamics: Study and apply kinematic models. For a robotic arm (perhaps a 2-DOF DIY arm), derive the forward kinematics and program it to move to given Cartesian points. Simulate or use a physics engine to see dynamics. If available, experiment with a robotic simulation (like using ROS and Gazebo with a sample robot model) to plan motions.
Robot Operating System (ROS): Dive into ROS by following tutorials. Build a mobile robot (or use a simulator like TurtleBot in Gazebo) and implement SLAM and navigation. Leverage existing ROS packages to create a map and navigate to goals. This will teach how to integrate perception and planning in a modular way.
Perception & Computer Vision: Learn OpenCV with Python or C++ to do tasks like image filtering, edge detection, simple object tracking. Then explore deep learning for vision: train a neural network (using a framework like PyTorch) to classify objects or detect lanes. On the sensor side, try working with LiDAR data – perhaps use PCL to filter a point cloud or identify planes.
SLAM and Localization: Implement or use an existing SLAM algorithm with a robot. Visualize the mapping process. You could start with 2D LiDAR SLAM (which is easier) and move to visual SLAM (harder due to data association issues). Understand the probabilistic nature of these algorithms.
Path Planning Algorithms: Write a simple A* path planner for a grid map to reinforce understanding. Then try using ROS’s move_base or MoveIt for planning and observe how the algorithms perform. Perhaps implement RRT for a planar scenario to see how sampling works.
Integration into Autonomous System: Finally, simulate an “autonomous vehicle” scenario – e.g., use CARLA or LGSVL simulators which integrate sensors, traffic scenarios, etc. Try to get your system to drive from point A to B obeying traffic lights in simulation. If you have access, participate in an autonomous driving challenge or build an RC car with the DonkeyCar or F1Tenth (1/10th scale autonomous car) platform – these are excellent hands-on ways to apply the full stack on a smaller scale.
Throughout this journey, keep the mathematical rigor in parallel with practical implementation. For each topic, there are excellent resources: textbooks like “Modern Robotics” by Kevin Lynch (for kinematics and control), “Probabilistic Robotics” by Thrun et al. (for SLAM and localization), and “Computer Vision: A Modern Approach” or online courses for vision, etc., can provide deep dives. Given the fast pace of advancement, also stay updated via research papers and forums for the latest in autonomous systems (many are open-access on arXiv). One also cannot underestimate the value of projects and competitions – building actual systems (however small) solidifies knowledge. Whether it’s a DIY drone, a micromouse maze solver, or a self-driving toy car, these projects teach integration and real-world problem solving, which is the essence of robotics engineering. Finally, remember that this field requires interdisciplinary thinking. An autonomous robot is at the intersection of mechanical systems, electronics, and software/AI. Communication skills and teamwork are important too, as large projects involve many specialists. As you progress from “bare metal” to “self-driving”, you’re essentially climbing layers of abstraction – never losing sight of the layers below. The motor won’t turn because a transistor burnt out, or the neural net won’t classify right because the lens got foggy – an autonomous systems engineer needs that holistic awareness from hardware to software. Good luck on your journey from blinking LEDs to autonomous robots – it’s a challenging but incredibly exciting ride. With patience and practice, you’ll be contributing to the next generation of intelligent systems that bridge the virtual and physical worlds. References: This guide has aggregated knowledge from numerous sources and references. Key standards and definitions were drawn from ISO and industry glossaries
github.com
en.wikipedia.org
. Foundational concepts in embedded systems and RTOS were referenced from textbooks and reputable blogs
en.wikipedia.org
github.com
. Robotics content was informed by educational roadmaps
erc-bpgc.github.io
erc-bpgc.github.io
 and classic literature in the field. For autonomous vehicles, industry guides and academic surveys provided insights into architecture and sensors
blackberry.qnx.com
blackberry.qnx.com
. Throughout, we cited specific sources inline (using the notation【†】) where detailed facts or definitions were provided, ensuring that you can consult those for deeper reading on each sub-topic if desired. This combination of sources represents the state-of-the-art understanding as of 2025 and should serve as a solid starting point for deeper exploration into each area. Enjoy building the future!
github.com
blackberry.qnx.com
Citations
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Bare machine - Wikipedia

https://en.wikipedia.org/wiki/Bare_machine
Favicon
Microcontroller vs Microprocessor: What's the Difference? | IBM

https://www.ibm.com/think/topics/microcontroller-vs-microprocessor
Favicon
Microcontroller vs Microprocessor: What's the Difference? | IBM

https://www.ibm.com/think/topics/microcontroller-vs-microprocessor
Favicon
Microcontroller vs Microprocessor: What's the Difference? | IBM

https://www.ibm.com/think/topics/microcontroller-vs-microprocessor
Favicon
Microcontroller vs Microprocessor: What's the Difference? | IBM

https://www.ibm.com/think/topics/microcontroller-vs-microprocessor
Favicon
Microcontroller vs Microprocessor: What's the Difference? | IBM

https://www.ibm.com/think/topics/microcontroller-vs-microprocessor
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
GitHub - m3y54m/Embedded-Engineering-Roadmap: Comprehensive roadmap for aspiring Embedded Systems Engineers, featuring a curated list of learning resources

https://github.com/m3y54m/Embedded-Engineering-Roadmap
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
What is a closed loop control system and how does it work?

https://www.techtarget.com/whatis/definition/closed-loop-control-system
What is a closed loop control system and how does it work?

https://www.techtarget.com/whatis/definition/closed-loop-control-system
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Robot kinematics | Intro to Autonomous Robots Class Notes - Fiveable

https://library.fiveable.me/introduction-autonomous-robots/unit-1/robot-kinematics/study-guide/jskgDTAHC4NWJCBU
Favicon
Robot kinematics - Wikipedia

https://en.wikipedia.org/wiki/Robot_kinematics
Favicon
Robot kinematics - Wikipedia

https://en.wikipedia.org/wiki/Robot_kinematics
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
A Visual Guide to the Software Architecture of Autonomous Vehicles | by Justin Milner | Medium

https://medium.com/@justinmilner/a-visual-guide-to-the-software-architecture-of-autonomous-vehicles-390b1744cbd6
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Roadmap - ERC Handbook

https://erc-bpgc.github.io/handbook/roadmap/
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
Favicon
Autonomous Systems | Ultimate Guides | BlackBerry QNX

https://blackberry.qnx.com/en/ultimate-guides/autonomous-systems
A Visual Guide to the Software Architecture of Autonomous Vehicles | by Justin Milner | Medium

https://medium.com/@justinmilner/a-visual-guide-to-the-software-architecture-of-autonomous-vehicles-390b1744cbd6
A Visual Guide to the Software Architecture of Autonomous Vehicles | by Justin Milner | Medium

https://medium.com/@justinmilner/a-visual-guide-to-the-software-architecture-of-autonomous-vehicles-390b1744cbd6
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/
Favicon
[Hello World!] Sense — Plan — Act – StreetDrone

https://www.streetdrone.com/hello-world-sense-plan-act/

## GitHub Pages
To view this roadmap as an interactive website, enable **GitHub Pages** in repository settings and select the `docs/` folder as the source.
