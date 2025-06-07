---
layout: default
title: Portfolio
description: Portfolio
---

<!-- # **Meet Coela**{: .typewriter } -->

Engineer and developer who solves problems with hardware, software, and automation. Building performant, modular, and reliable systems-from embedded devices and robotics to virtualization and infrastructure automation.

## **Software and Embedded Systems Development**
<!-- ProtoTracer -->
{% project %}
name: ProtoTracer

summary: "ProtoTracer: Embedded 3D Rendering Engine"

links:
  - name: Website
    url: https://coelacant.com/ProtoTracer/
  - name: GitHub
    url: https://github.com/coelacant1/ProtoTracer

dates: "2020 - Now"

contributors: "Contributors on GitHub"

description: |-
  ProtoTracer is a C++ based 3D rendering engine optimized for microcontrollers. It calculates dynamic 3D scenes in real-time, reacting to sensor or user inputs to adapt the output. This software is designed to be modular for use in other projects.
  
  This includes features such as: 
  - Keyframed animations with support for automated easing
  - Display of custom converted FBX, OBJ 3D Models with support for textures using any static image format as well as support for animated GIFs
  - Custom camera definitions and parameters for specialized display shapes and 3D outputs
  - Support for driving HUB75, WS2812, and APA102 based LED displays, with controller definitions making it easy to create other interfaces
  - Raster-based 3D rendering as well as Raytraced rendering
    - Segmented-rendering optimization using quadtrees
    - Custom rendering pipeline optimized for high frame rates on constrained hardware
      - Enables PS1-level graphics rendering
  - Custom static and animated shader materials
    - Simplex Noise Shaders
    - Audio Reactive Shaders (Oscilloscope/Spectrum Analyzer)
    - TV Static emulation
    - Gradient/Normal/UV Tiled/Depth Materials
  - Audio Analysis
    - Voice analysis for matching vowel sounds to viseme shapes
      - Allows user calibration for custom formant maps
    - Fourier Transform for analysis/spectrum analyzer display
    - Filtering and automatic scaling for background noise
    - Filters: FFT/Kalman/Peak/Ramp/Running Average
  - Physics Simulator
  - Hardware Support for:
    - SSD1306 Status Display/HUD
    - Communication with secondary controller for wireless control
    - PWM Fan Controls through a user menu
    - APDS9960 Color/Distance Sensing
    - BNO055 Quaternion input for space-mouse like control
    - MMC56X3 Magnetometer input
    - SHARPGP2Y Distance Sensing
    - MAX9814/SPW2430 Microphone Input
  - Screenspace Shaders for manipulating 3D rendered frames with 2D modifications
    - Fisheye distortion
    - Glitch distortion
    - Box/Radial blur
    - Magnetic Lens distortion
    - Phase Shifting
  - Live object manipulation
    - Position/Scale/Rotation
    - Distortion with custom object deformer
      - Modifies 3D space via transformation functions
      - Perspective Deform
      - Sinusoidal Deform
      - Dropwave Deform
    - Automatic 3D object alignment
      - Allows alignment of center of volume or mass to a target coordinate
      - Fit alignment to a plane
        - Allows users to rapidly add their own model with any scale/rotation and it will autofit to a target size and plane
  - Custom Optimized Math Library
    - Rotation library with support for Axis Angles, Direction Angles(Custom), Euler Angles, Quaternions, Rotation Matrices, Yaw-Pitch-Roll
    - Vector2D and Vector3D Library
    - Quaternion Library
    - Kalman and Running Average Filters for Quaternion Space, Cartesian Space
  - Automated Testing to verify custom math libary
    
  Doxygen genereated documentation from sourcecode:

iframe: https://coelacant.com/ProtoTracer/

slides:
  - type: video
    video_id: ii_2rAg5L5c
    caption: "ProtoTracer menu using multi-button RGB I2C Controller"
  - type: video
    video_id: xQfBLp0Ws1g
    caption: "3D shader running a 2D rasterizer that is then modified with simulated advection"
  - type: video
    video_id: C5WwMnbaJ34
    caption: "Shader with multiply blend modes between two animated GIFs"
  - type: video
    video_id: 8uSgCMLcty8
    caption: "Simplex noise shader with blend mode mixing stripe shader"
  - type: video
    video_id: 1AH0N9evXL4
    caption: "Spiral rainbow shader with multiply blend mode playing Bad Apple as a GIF"
  - type: video
    video_id: SXp2C7LbiRM
    caption: "Dual spiral rainbow shader with lighten blend mode"
  - type: video
    video_id: Yc67nFJwqYo
    caption: "Example of a full keyframed animation"
  - type: video
    video_id: TnoXMWCxYpc
    caption: "External sensor input for controlling the 3D model"
  - type: video
    video_id: ucIHCd_vjGA
    caption: "Rendering a 3D model with 2D textures"
  - type: video
    video_id: ZJqEkMUVPH4
    caption: "Normal shader"
  - type: video
    video_id: orijCE6EDM4
    caption: "Physics simulator with balls in a 3D box"
  - type: video
    video_id: Xh3eKCqLIPc
    caption: "Example of a custom software build for a user"
  - type: pdf
    src: /assets/pdfs/ProtogenControllerV2-0.pdf
    caption: "Controller guide for hardware optimized for ProtoTracer"

{% endproject %}

<!-- AetherControl -->
{% project %}
name: AetherControl

summary: "AetherControl: Optimized CNC Firmware"

links:
  - name: GitHub
    url: https://github.com/coelacant1/AetherControl

dates: "2023 - Now"

contributors: "Solo Project"

description: |-
  AetherControl is a streamlined and modular, C++ based motion-control firmware. It is built for real-time G-code parsing, motion planning, kinematics, and stepper-motor drive. Supports advanced kinematics like Stewart platforms, CoreXY machines, and Cartesian robots with dynamic trajectory planning.

  Implementations:
  - High-Speed CoreXY pick-and-place machine
  - High-Speed 6x 1000W AC Servo based 6-axis Stewart platform used as a flight simulator

  This includes features such as:
  - Support for Stewart platform, Cartesian, CoreXY, and custom kinematics configurations
  - Dynamic trajectory planning with velocity and acceleration constraints
  - G-code parsing and execution for CNC command interpretation
  - Real-time multi-axis synchronization and motion coordination
  - Multi-Controller synchronization
  - Layered modular architecture for easy customization and extension
  - Support for sensor calibration and automated testing routines
  - Optimized for ARM Cortex-M microcontrollers with efficient resource use
  - Custom PathPlanner optimizing for up to the number of GPIO pins available to drive the system

slides:
  - type: video
    video_id: 4LFMXw97F7c
    caption: "AetherControl on a CoreXY Pick n Place Machine"
  - type: image
    src: /assets/images/AetherControl1.jpg
    caption: "Example configuration for the AB motors of the Pick n Place"
    thumb: /assets/images/thumbs/AetherControl1.jpg
  - type: video
    video_id: nXmGRJHBArY
    caption: "Second example of AetherControl on the CoreXY Pick n Place assembling a PCB"
  - type: image
    src: /assets/images/AetherControl2.jpg
    thumb: /assets/images/thumbs/AetherControl2.jpg
    caption: "Example configuration for the Z and R motors of the Pick n Place - end effector controller"
  - type: video
    video_id: DOIEZTeIsqk
    caption: "AetherControl controlling a 6-Axis 425mm stroke Stewart Platform"
  - type: image
    src: /assets/images/AetherControl3.jpg
    thumb: /assets/images/thumbs/AetherControl3.jpg
    caption: "Configuration file for the Stewart Platform"

{% endproject %}

<!-- ProxmoxScripts -->
{% project %}
name: ProxmoxScripts

summary: "ProxmoxScripts for Automated Infrastracture Management"

links:
  - name: Website
    url: https://coelacant.com/ProxmoxScripts/
  - name: GitHub
    url: https://github.com/coelacant1/ProxmoxScripts

dates: "2024 - Now"

contributors: "Contributors on GitHub"

description: |-
  ProxmoxScripts is a collection of Bash scripts for streamlining and automating the management of Proxmox Virtual Environment (PVE) clusters. This was developed to help manage and automate Proxmox virtualization infrastructure. The scripts are accessible via a console-based menu for overview, navigation, and execution. Users can read help information for each script as well as get a description of required parameters.

  Features include: 
  - Cluster configuration tools
    - Adding Nodes - Remote configuration with IP
    - Cluster Creation - From list of IPs
    - Cluster Deletion - Separating all nodes from cluster
  - Firewall management
    - Bulk LXC/VM Firewall Management
    - Automated host default firewall configuration
  - High Availability
    - Create HA Group and add nodes 
    - Bulk add LXC/VMs to HA group
  - Host Automation and Configuration
    - Remove local-lvm storage and expand local storage (for Hyper-Converged setups)
    - Bulk Microcode configuration/Timezone Configuration/Upgrades
    - Generic PWM and Dell IPMI hardware fan control adapting to CPU temperatures
    - Enable CPU Scaling Governer
    - Enable GPU Passthrough to VM
    - Enable IOMMU
    - Enable GPU Sharing with LXC
    - Optimize for X3D hardware
    - Live memory testing
    - System optimization for nested virtualization
  - LXC and VM Configuration and Management
  - Networking
    - Bulk adding network bonds
    - List all VM/LXC IDs with their associated MAC address
    - Bulk set DNS on cluster
    - Find VM ID from MAC address
    - Automated host-to-host network speed test
    - Bulk update network interface names 
    - Bulk host uplink speed test
  - Remote VM/LXC Management
    - Integration with Apache Guacamole
      - Bulk add/delete RDP connections
      - Bulk add/remove SFTP server to RDP connections
      - Bulk list RDP configuration
      - Bulk update/remove drive redirection
      - Pull Guacamole authentication token
    - Configuration of virtual machine static IPs over SSH for Debian/Ubuntu/Windows
  - Generate a resource report for all VMs/LXCs
  - Automated penetration testing/port scanning
  - Storage Automation
    - Ceph Configuration
      - Bulk create OSDs
      - Edit crushmap (decompiles, prints directory, allows user edit, then recompile)
      - Bulk restart managers/monitors/metadata servers/OSDs
      - Set pool to minimum size of 1 and to a size of 1 (testing/temporary data)
      - Create automated deep scrubbing with a chron job
      - Automate creating a single drive Ceph node (shares host OS and Ceph data)
      - Bulk OSD sparsify
      - Bulk start stopped disks
      - Full data wipe on specified disk (clear Ceph and all remnants of user data)
    - Disk Benchmarking
    - Bulk VM disk deletion
    - Bulk VM disk deletion for disks with snapshots
    - Trim all filesystems of resources using Qemu Agent or LXC
    - Automate disk spin down
    - Pass host directory to LXC
    - Update stale mounts
  - Custom User Interface library

iframe: https://coelacant.com/ProxmoxScripts/

slides:
  - type: image
    src: /assets/images/ProxmoxScripts5.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts5.jpg
    caption: "Screenshot of the custom terminal user interface for CCPVE"
  - type: image
    src: /assets/images/ProxmoxScripts4.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts4.jpg
    caption: "Example of running a command/script available through CCPVE"
  - type: image
    src: /assets/images/ProxmoxScripts2.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts2.jpg
    caption: "Example of automatically cloning, SSHing to the Windows VM, configuring the IP and cloning the next VM"
  - type: image
    src: /assets/images/ProxmoxScripts1.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts1.jpg
    caption: "Example of batch cloning with CCPVE"

{% endproject %}

<!-- OpenDACT -->
{% project %}
name: OpenDACT

summary: "OpenDACT: Automatic Delta Kinematics Calibration Software"

links:
  - name: Github
    url: https://github.com/coelacant1/OpenDACT

dates: "2014 - 2017"

contributors: "Contributors on GitHub"

description: |-
  OpenDACT (Open-source Delta Automatic Calibration Tool) is a calibration tool for delta robots. It calibrates generic delta robots and automates calibration of Repetier-firmware-controlled delta robots (primarily 3D printers). The calibration uses forward and inverse kinematics measurements to make adjustments. It takes Z-height readings at key XY points on a flat plate (print bed) to iteratively calculate hardware offsets. Users can manually adjust hardware or apply software offsets to correct for imperfections.

  This software solves for the following mechanical offsets:
  - Diagonal rod lengths
  - End effector radius
  - Horizontal radius
  - Endstop offsets
  - Angles of the ABC towers

  For Repetier based robots, these offsets are automatically saved on the controllers EEPROM upon a successful calibration. Z-bed distortion on a newly completed build could have offsets of +/- 2mm across the plate, this tool (assuming issues in the above offsets) can calibrate the systems down to 5-10 microns.
  
  This software has been tested and successfully used on small-form factor deltas (100mm plate diameter) and large-form factor deltas (1.5 meter plate diameter).

slides:
  - type: image
    src: /assets/images/OpenDACT1.jpg
    thumb: /assets/images/thumbs/OpenDACT1.jpg
    caption: "User interface with example calibration of a delta 3D printer running Repetier firmware"
  - type: image
    src: /assets/images/OpenDACT2.jpg
    thumb: /assets/images/thumbs/OpenDACT2.jpg
    caption: "User interface showing deformation in the physical structure from ideal conditions - can be used to physically tune the machine"
  - type: image
    src: /assets/images/OpenDACT4.jpg
    thumb: /assets/images/thumbs/OpenDACT4.jpg
    caption: "Full user interface while running a calibration"

{% endproject %}

<!-- Proxmox Load Balancer -->
{% project %}
name: PVELoadBalancer

summary: "Proxmox VE Load Balancer"

links:
  - name: GitHub
    url: https://github.com/coelacant1/ProxmoxLoadBalancer

dates: "2025 - Now"

contributors: "Solo Project"

description: |-
  ProxmoxLoadBalancer is a Python tool that balances memory across Proxmox VE cluster nodes. It has two modes of operation:

  - Simulation Mode: Test and visualize load-balancing algorithms in a controlled environment without touching the live cluster.
  - Live Mode: Uses the Proxmox API to monitor memory usage and migrate VMs to balance load.

  This software is built as a stand-in until ProxmoxVE ships with a built-in cluster load balancer.

iframe: https://coelacant.com/ProxmoxScripts/

slides:
  - type: image
    src: /assets/images/PVELoadBalancer3.jpg
    thumb: /assets/images/thumbs/PVELoadBalancer3.jpg
    caption: "Example simulated load balance of a 12-node ProxmoxVE cluster with different amounts of memory"
  - type: image
    src: /assets/images/PVELoadBalancer1.jpg
    thumb: /assets/images/thumbs/PVELoadBalancer1.jpg
    caption: "Example moves from the simulation to balance to equally shared memory"
  - type: image
    src: /assets/images/PVELoadBalancer2.jpg
    thumb: /assets/images/thumbs/PVELoadBalancer2.jpg
    caption: "Second example of a simulated load-balance but with more extremes in initial conditions"
  - type: image
    src: /assets/images/PVELoadBalancer4.jpg
    thumb: /assets/images/thumbs/PVELoadBalancer4.jpg
    caption: "Combined simulation of initial load, final load, and moves to get to final load"

{% endproject %}

<!-- Teensy WebHID -->
{% project %}
name: WebHID

summary: "Teensy WebHID Firmware Loader"

dates: "2025 - Now"

contributors: "Solo Project"

links:
  - name: Example Page
    url: https://coelacant.com/Teensy-Loader-Javascript/Teensy-Loader-Example.html
  - name: Github
    url: https://github.com/coelacant1/Teensy-Loader-Javascript

description: |-
  The Teensy WebHID Loader is a Chromium-based firmware flasher replicating PJRC's Teensy Loader. It parses and uploads HEX and BIN files to Teensy microcontrollers, manages serial connections, and lets end users update device firmware without installing any software.

iframe: https://coelacant.com/Teensy-Loader-Javascript/Teensy-Loader-Example.html

slides:
  - type: image
    src: /assets/images/TeensyWebHID7.jpg
    thumb: /assets/images/thumbs/TeensyWebHID7.jpg
    caption: "Example of the Teensy WebHID loader integrated into a site for end user firmware updates"
  - type: image
    src: /assets/images/TeensyWebHID2.jpg
    thumb: /assets/images/thumbs/TeensyWebHID2.jpg
    caption: "List of available .hex files to upload to the microcontroller"
  - type: image
    src: /assets/images/TeensyWebHID3.jpg
    thumb: /assets/images/thumbs/TeensyWebHID3.jpg
    caption: "Device selection"
  - type: image
    src: /assets/images/TeensyWebHID4.jpg
    thumb: /assets/images/thumbs/TeensyWebHID4.jpg
    caption: "Uploading firmware to a Teensy 4.0 microcontroller with the WS2812 3.5mm panel configuration"
  - type: image
    src: /assets/images/TeensyWebHID5.jpg
    thumb: /assets/images/thumbs/TeensyWebHID5.jpg
    caption: "Firmware upload completed"
  - type: image
    src: /assets/images/TeensyWebHID6.jpg
    thumb: /assets/images/thumbs/TeensyWebHID6.jpg
    caption: "Serial output from the microcontroller into the browser"

{% endproject %}

<!-- Proxmox GPU Passthrough Optimizations -->
{% project %}
name: ProxmoxGPU

summary: "Proxmox GPU Passthrough for Workstations"

links:
  - name: GitHub
    url: https://github.com/coelacant1/PVEGPUPassthroughGuide

dates: "2025 - Now"

contributors: "Solo Project"

description: |-
  A guide to running Proxmox VE as the base operating system for a high-performance workstation. Covers GPU and USB controller passthrough; multi-NUMA-node CPU optimization; BIOS and GRUB tweaks; VFIO module setup and driver blacklisting; microcode updates and GPU ROMs; tuning CPU affinity; network-stack optimizations; Windows/QEMU adjustments for Easy Anti-Cheat compatibility; example VM setups; and benchmarking results.
  
  This enables users to run high-performance workstations in various configurations while keeping things easy to manage, since each VM can be tuned for its specific OS or software.

slides:
  - type: pdf
    src: /assets/pdfs/PVEGPUPassthroughGuide.pdf
    caption: "The complete PDF guide from the Github markdown files"

{% endproject %}

## **Product Prototyping and Engineering**
<!-- Stewart Platform -->
{% project %}
name: StewartPlatform

summary: "Stewart Platform - Firmware and Hardware for Flight Simulator"

dates: "2020 - Now"

contributors: "Hardware/Software Solo | Research by Co-workers"

links:
  - name: Github
    url: https://github.com/coelacant1/StewartPlatformIK

description: |-
  The Stewart Platform project is an adaptation of a commercial system. The machine originally shipped with MDBox motion controllers, controlled via ethernet. Because of high latency it was adapted with a custom controller, upgraded 1000W AC servos for the linear actuators, and custom firmware and software. This was the start of the AetherControl project detailed above. The attached repository contains the interface code that takes target XYZ Cartesian coordinates and Euler rotations, then computes the six actuator lengths for AetherControl. The goal of the project was to adapt the system to handle high-frequency vibration and high-speed motion for research on flight dynamics.

  The inverse kinematics repository allows for custom user-defined configuration parameters, taking in:
  - Base Plate Radius
  - Base Plate Mounting Angle
  - Platform Plate Radius
  - Platform Mounting Angle
  - Base Height
  - Maximum Actuator Length

  Users can input an XYZ cartesian coordinate and Yaw-Pitch-Roll to get a returned value of each of the 6 actuator lengths.

slides:
  - type: video
    video_id: DOIEZTeIsqk
    caption: "Test motion of the stewart platform to validate controls"
  - type: video
    video_id: CeZdBrVZPds
    caption: "Stewart platform tied to Microsoft flight simulator"
  - type: video
    video_id: xXMPqDMNq30
    caption: "Test motion of the stewart platform"
  - type: image
    src: /assets/images/StewartPlatform3.jpg
    thumb: /assets/images/thumbs/StewartPlatform3.jpg
    caption: "Stewart platform in the default position prior to mounting the simulator seat"
  - type: image
    src: /assets/images/StewartPlatform2.jpg
    thumb: /assets/images/thumbs/StewartPlatform2.jpg
    caption: "Updated controller, switched from slow MDBox controllers (100ms max update rate) to 6x 1000W AC servos for real-time control"
  - type: image
    src: /assets/images/StewartPlatform1.jpg
    thumb: /assets/images/thumbs/StewartPlatform1.jpg
    caption: "Custom controller for controlling the previous MDBox controls as well as the new step/direction based input using a Teensy 4.1"
{% endproject %}

<!-- Pick n Place -->
{% project %}
name: PicknPlace

summary: "Custom High-Speed CoreXY Pick-and-Place"

dates: "2021 - current"

contributors: "Solo Project"

links:
  - name: Github
    url: https://github.com/coelacant1/CoelaPNP

description: |-
  This project started was to create a small yet capable Pick-and-Place machine. This machine has been used to manufacture RGB LED displays which each have over 500 LEDs, power distribution boards, custom LED control electronics, etc.
  
  Features:
  - OpenPNP for software control
    - This system supports most of the hardware calibration features provided by OpenPNP
  - CoreXY design based on the Voron project's motion system
  - Closed-loop AB motors for X and Y axes
  - Feedrates up to 800mm/s
  - Accelerations up to 40m/s^2
  - 12 Pneumatic feeders
  - Dual controller design
    - Primary controller for controlling closed-loop AB motors for the CoreXY motion system
    - Secondary controller for integrated USB hub to connect cameras/controller, Z and rotation axis stepper motor control with TMC2209 stepper drivers
  - 120 fps cameras with fixed exposure and white-balance settings
  - Fiducial camera for alignment of X and Y positions as well as fine-tuning the motion system backlash and pick-up locations
  - Part camera for alignment of parts/components

slides:
  - type: video
    video_id: 2Rnxu-sZ-2k
    caption: "Vision system and part pick up test"
  - type: video
    video_id: c8iBtan-B90
    caption: "Automatic 3D nozzle calibration and alignment"
  - type: video
    video_id: EPM9clNw89M
    caption: "Homing sequence using optical endstops and an optical based precision alignment using a fiducial"
  - type: video
    video_id: wRKHLm215EY
    caption: "Test assembly of a custom RGB led board"
  - type: video
    video_id: moOtE-V9jf4
    caption: "3D calibration and close up to show precision of the machine"
  - type: video
    video_id: bi5gHuZIw6Y
    caption: "Example assembly prior to tuning the vision system"
  - type: video
    video_id: LeFAltqtTKM
    caption: "Testing the upgraded end effector"
  - type: video
    video_id: PrOzkztaJ80
    caption: "High speed and high acceleration motion testing - 40m/s2 acceleration and 600-700mm/s"
  - type: video
    video_id: 5-n5m6Ypd6U
    caption: "Prototype pick n place built with hardware from other projects"
  - type: image
    src: /assets/images/PnP3.jpg
    thumb: /assets/images/thumbs/PnP3.jpg
    caption: "Bottom of the PnP prior to wiring and air runs"
  - type: image
    src: /assets/images/PnP7.jpg
    thumb: /assets/images/thumbs/PnP7.jpg
    caption: "Boards that were assembled with the pick n place - WS2812 LED boards with 571 LEDs per board"
  - type: image
    src: /assets/images/PnP9.jpg
    thumb: /assets/images/thumbs/PnP9.jpg
    caption: "Close up of the LED boards"
  - type: image
    src: /assets/images/PnP20.jpg
    thumb: /assets/images/thumbs/PnP20.jpg
    caption: "Wiring and air runs completed on the pick n place"
  - type: image
    src: /assets/images/PnP10.jpg
    thumb: /assets/images/thumbs/PnP10.jpg
    caption: "Validating the control signals for the motion system to check if impedance matching for wiring was correct"
  - type: image
    src: /assets/images/Electronics29.jpg
    thumb: /assets/images/thumbs/Electronics29.jpg
    caption: "HUB75 and WS2812 based RGB LED controllers assembled on pick n place"
  - type: image
    src: /assets/images/Electronics24.jpg
    thumb: /assets/images/thumbs/Electronics24.jpg
    caption: "Close up of the LED boards"
  - type: image
    src: /assets/images/Electronics12.jpg
    thumb: /assets/images/thumbs/Electronics12.jpg
    caption: "Close up of the Pick n Place controller"
  - type: image
    src: /assets/images/PnP4.jpg
    thumb: /assets/images/thumbs/PnP4.jpg
    caption: "Pick n Place frame assembly on first iteration"
  - type: image
    src: /assets/images/PnP14.jpg
    thumb: /assets/images/thumbs/PnP14.jpg
    caption: "Pick n Place CoreXY belt mounting and end-effector"
  - type: image
    src: /assets/images/PnP12.jpg
    thumb: /assets/images/thumbs/PnP12.jpg
    caption: "Upgraded Closed-Loop stepper motors driven at 48V for fast saturation and higher accelerations"
  - type: image
    src: /assets/images/PnP15.jpg
    thumb: /assets/images/thumbs/PnP15.jpg
    caption: "Custom upgraded end effector controller for the pick n place"
  - type: image
    src: /assets/images/PnP16.jpg
    thumb: /assets/images/thumbs/PnP16.jpg
    caption: "Assembled end effector controller for controlling the Z, rotation axis, as well as a USB 2.0 MSTP hub for the cameras and micro controller"
  - type: image
    src: /assets/images/PnP22.jpg
    thumb: /assets/images/thumbs/PnP22.jpg
    caption: "CAD of the first PnP iteration"
{% endproject %}

<!-- Delta 3D Printer -->
{% project %}
name: Delta3D

summary: "Custom High-Speed Delta 3D Printer"

dates: "2014 - 2019"

contributors: "Solo Project"

description: |-
  This project started in the era of the Prusa Mendel/i3 designs. Required faster print times than available 3D printers in the market, designed and built this system to solve that problem.

  Features:
  - 270mm build diameter
  - ~490mm build height
  - Printing travel speeds up to 500mm/s
  - Printing speeds with extrusion up to 150mm/s (layer height dependent)
  - Accelerations up to 10m/s^2
  - 32-bit controller using Smoothieware/Marlin (previously 8-bit with Repetier)
  - Custom frame and design
  - Custom, compact end-effector and extruder combo
  - Automatic bed leveling using FSRs integrated into the bed mounts

slides:
  - type: image
    src: /assets/images/Delta8.jpg
    thumb: /assets/images/thumbs/Delta8.jpg
    caption: "Picture of the Delta 3D printer alongside a 180mmx180mmx180mm 3D printer"
  - type: image
    src: /assets/images/Delta7.jpg
    thumb: /assets/images/thumbs/Delta7.jpg
    caption: "Upgraded end-effector system with dual gear drive for the extruder"
  - type: image
    src: /assets/images/Delta5.jpg
    thumb: /assets/images/thumbs/Delta5.jpg
    caption: "Force-Sensing Resistors for bed calibration, using built in sliders that constrain the motion and prevent the bed from falling off"
  - type: image
    src: /assets/images/Delta3.jpg
    thumb: /assets/images/thumbs/Delta3.jpg
    caption: "Belt structure for the upgraded extruder"
  - type: image
    src: /assets/images/Delta1.jpg
    thumb: /assets/images/thumbs/Delta1.jpg
    caption: "Delta frame prior to upgrading the power system and wiring"
  - type: video
    video_id: Am7Ycc9Rg_M
    caption: "First test print with the upgraded delta"
  - type: video
    video_id: k7pzi0IPiGI
    caption: "Normal/low print speed test with the previous extruder design"
  - type: video
    video_id: LGvfJACLQG8
    caption: "Test print of a vacuum forming buck"
  - type: video
    video_id: 7CEiR0AMyPc
    caption: "Final print at low-speed of the vacuum forming buck"

{% endproject %}

<!-- 4-Axis Cutter + Fusion Post Processor -->
{% project %}
name: 4AxisCutter

summary: "Custom 4-Axis Machines and Fusion 360 Post Processors"

links:
  - name: Github
    url: https://github.com/coelacant1/4-Axis-Fusion360-Post-Processor

dates: "2023 - 2024"

contributors: "Solo Project"

description: |-
  This project primarily is a custom-built Fusion 360 post-processor for multi-axis laser cutting and engraving. It was created to cut complex shapes into vacuum-formed plastic visors. The software is a Fusion 360 post-processor that uses the machine's hardware configuration to convert 4-axis toolpaths into G-code for a custom 4-axis laser cutter.

  The machine is an XYZ motion system with an additional large rotary axis that has an integrated fume exhaust system.

slides:
  - type: image
    src: /assets/images/4Axis3.jpg
    thumb: /assets/images/thumbs/4Axis3.jpg
    caption: "Final assembly of the 4-axis laser cutter"
  - type: image
    src: /assets/images/4Axis1.jpg
    thumb: /assets/images/thumbs/4Axis1.jpg
    caption: "Rear-view of the laser cutter"
  - type: image
    src: /assets/images/4Axis2.jpg
    thumb: /assets/images/thumbs/4Axis2.jpg
    caption: "Wiring for the laser cutter"
  - type: video
    video_id: 26WvXgYY3GE
    caption: "Simulation using a custom Fusion 360 post processor"
  - type: video
    video_id: vq9g52vjKNk
    caption: "Testing the laser cutter (without power on the diode)"

{% endproject %}

<!-- 4-Axis Automated Gimbal -->
{% project %}
name: 4AxisGimbal

summary: "4-Axis Automated Gimbal for MEMS-Based Motion Processor Testing"

links:
  - name: Github
    url: https://github.com/coelacant1/GimbalSimulator

dates: "2018 - 2020"

contributors: "Solo Project"

description: |-
  The 4-axis gimbal is the primary tool in a larger project. This project required accurate calibration of low-cost MPU6050 motion sensors. To perform the calibration a custom 4-axis motion system (3 rotational axes and 1 linear axis) closely follows a path while capturing the data (acceleration/angular velocity and for some also the magnetic field) from multiple inertial measurement units. A separate tool-the linked GitHub repository-simulates the system and generates ideal theoretical sensor outputs. Comparing real and simulated data is used to calibrate the system and develop filters for the sensors adjusted outputs. 

slides:
  - type: video
    video_id: eGvnqaCpQ50
    caption: "Homing sequence and test move with the assembled motion system"
  - type: video
    video_id: KaQvgXYtTIc
    caption: "Live output of the 4 accelerometers and 4 gyroscopes - this data is captured for the calibration process"
  - type: video
    video_id: degD95T-_Mg
    caption: "Rendered path of a travel move"
  - type: image
    src: /assets/images/Gimbal11.jpg
    thumb: /assets/images/thumbs/Gimbal11.jpg
    caption: "Side view of the gimbal"
  - type: image
    src: /assets/images/Gimbal9.jpg
    thumb: /assets/images/thumbs/Gimbal9.jpg
    caption: "MPU6050 reading hardware"
  - type: image
    src: /assets/images/Gimbal8.jpg
    thumb: /assets/images/thumbs/Gimbal8.jpg
    caption: "Quarter view of the gimbal"
  - type: image
    src: /assets/images/Gimbal7.jpg
    thumb: /assets/images/thumbs/Gimbal7.jpg
    caption: "Top view"

{% endproject %}

<!-- Custom Electronics Design -->
{% project %}
name: CustomElectronics

summary: "Power Electronics and Control Systems"

dates: "2014 - Now"

contributors: "Solo Projects"

description: |-
  Designed and built custom power-conversion, motion‑control systems, and other electronics:
  - Custom Buck Power Regulators: Gallium nitride-based buck converters stepping 60 V down to 5 V at up to 20 A
  - LoRa Remote Firework Igniter: 12-channel, 2 A LoRa-based firework-igniter controller prototype
  - Haptics Research Motor Driver: DC motor driver with sensor feedback for inverted-pendulum and force-feedback experiments
  - High‑Speed Pick‑and‑Place End-Effector Controller: High-speed pick-and-place end-effector controller: board with onboard power regulation, a 4-port USB hub for cameras and motion control, and two TMC2209 stepper drivers
  - Stroboscope: High‑frequency LED stroboscope for motion analysis and AC/DC motor testing.
  - Controller for High-Speed Pick‑and‑Place: CNC control board handling six stepper motors, twelve MOSFET channels for pneumatics, and integrated power regulation
  - 8‑Channel HUB75 + WS2812 LED Matrix Driver: LED controller integrating HUB75 logic (row/column multiplexing) and 8-channel 500 count addressable strips
  - Wireless WS2812 LED Strip Controller: ESP32 and Teensy 4.0‑based 8-Channel LED Strip controller - able to drive up to 4,000 LEDs
  - ESP8266 + BME280 Environment Monitor: ESP8266 + BME280 environmental monitor: wireless temperature and humidity logger with web dashboard
  - SMT PCB Panelization Workflow: Panelized PCBs for small‑batch pick‑and‑place and reflow assembly

slides:
  - type: video
    video_id: Jkx1nAE-rY4
    caption: "Pick n Place end effector controller controlling a Nema 8 and Nema 17 motor"
  - type: video
    video_id: pgWeXkJTqyQ
    caption: "HUB75 and WS2812 RGB controller controlling PWM fans"
  - type: video
    video_id: SAm6Xv9psXs
    caption: "Pick n Place controller testing the mosfets for the vacuum and air control system as well as controlling a stepper motor"
  - type: image
    src: /assets/images/Electronics27.jpg
    thumb: /assets/images/thumbs/Electronics27.jpg
    caption: "Close up of the end-effector controller for the PnP"
  - type: image
    src: /assets/images/Electronics29.jpg
    thumb: /assets/images/thumbs/Electronics29.jpg
    caption: "Assembled RGB HUB75 and WS2812 LED controllers"
  - type: image
    src: /assets/images/Electronics28.jpg
    thumb: /assets/images/thumbs/Electronics28.jpg
    caption: "6-axis Stewart Platform controller with level shifting from 3.3V to 5V"
  - type: image
    src: /assets/images/electronics10.jpg
    thumb: /assets/images/thumbs/electronics10.jpg
    caption: "View in KiCAD of the PnP end-effector controller"
  - type: image
    src: /assets/images/electronics11.jpg
    thumb: /assets/images/thumbs/electronics11.jpg
    caption: "Transparent ayer view of the end-effector controller"
  - type: image
    src: /assets/images/Electronics34.jpg
    thumb: /assets/images/thumbs/Electronics34.jpg
    caption: "Close up of the PnP controller"
  - type: image
    src: /assets/images/Electronics32.jpg
    thumb: /assets/images/thumbs/Electronics32.jpg
    caption: "Wireless environmental monitor using ESP8266 microcontrollers and a BME280"
  - type: image
    src: /assets/images/Electronics30.jpg
    thumb: /assets/images/thumbs/Electronics30.jpg
    caption: "8-Channel WS2812 controller with support for wireless communication"
  - type: image
    src: /assets/images/Electronics25.jpg
    thumb: /assets/images/thumbs/Electronics25.jpg
    caption: "Close up of the HUB75 and 8-Channel WS2812 controller"
  - type: image
    src: /assets/images/Electronics19.jpg
    thumb: /assets/images/thumbs/Electronics19.jpg
    caption: "Board verification after assembly"
  - type: image
    src: /assets/images/Electronics8.jpg
    thumb: /assets/images/thumbs/Electronics8.jpg
    caption: "12-Channel firework ignitor using LoRa"
  - type: image
    src: /assets/images/Electronics5.jpg
    thumb: /assets/images/thumbs/Electronics5.jpg
    caption: "Compact 5V 20A Buck Converter using Gallium Nitride FETs"
  - type: image
    src: /assets/images/Electronics3.jpg
    thumb: /assets/images/thumbs/Electronics3.jpg
    caption: "Electronics for a stroboscope"
  - type: image
    src: /assets/images/Electronics7.jpg
    thumb: /assets/images/thumbs/Electronics7.jpg
    caption: "more complete view of the stroboscope hardware"
    caption: ""
  - type: image
    src: /assets/images/PCBProcess1.jpg
    thumb: /assets/images/thumbs/PCBProcess1.jpg
    caption: "Process for the PNP end-effector controller in Altium"
  - type: image
    src: /assets/images/PCBProcess2.jpg
    thumb: /assets/images/thumbs/PCBProcess2.jpg
    caption: "Process for the PNP end-effector controller in Altium"
  - type: image
    src: /assets/images/PCBProcess3.jpg
    thumb: /assets/images/thumbs/PCBProcess3.jpg
    caption: "3D view for the PNP end-effector controller in Altium"
  - type: image
    src: /assets/images/PnP19.jpg
    thumb: /assets/images/thumbs/PnP19.jpg
    caption: "Pick n Place controller in the machine"
  - type: pdf
    src: /assets/pdfs/pnpeec.pdf
    caption: "Layer view of the end-effector controller"
  - type: pdf
    src: /assets/pdfs/PNPEndEffector.pdf
    caption: "Schematic for the end-effector controller"
  - type: pdf
    src: /assets/pdfs/Stroboscope.pdf
    caption: "Schematic for the stroboscope, simulated in Altium"
  - type: pdf
    src: /assets/pdfs/ProtoController.pdf
    caption: "8-Channel WS2812 controller schematic"
  - type: pdf
    src: /assets/pdfs/PNPController.pdf
    caption: "Pick n Place Controller schematic"

{% endproject %}

<!-- High-Speed CoreXY 3D Printer -->
{% project %}
name: CustomVoron

summary: "Customized Voron CoreXY 3D Printer"

dates: "2019 - 2021"

contributors: "Solo Project"

description: |-
  A custom 3D printer derived from the Voron project, built to explore the limits of FDM additive manufacturing.

  - Extrusion and Motion Hardware
    - BondTech LGX extruder with Mosquito Magnum hotend for high flow
    - 48V powered stepper drivers with accelerations up to 2g (20m/s^2) and travel speeds near 600 mm/s
  - Reinforced Structure
    - Continuous carbon‑fiber-reinforced parts printed on the Markforged Mk2, plus a custom carbon‑fiber X‑gantry
  - Firmware and Calibration
    - Klipper firmware with computation offloaded to a Raspberry Pi
    - Custom-tuned to maximize performance within the system's limits

slides:
  - type: video
    video_id: Vec_Udgmh2Y
    caption: "Independent tower-leveling and bed calibration"
  - type: video
    video_id: G0I2Uv52RyM
    caption: "First dry-run print"
  - type: image
    src: /assets/images/Voron1.jpg
    thumb: /assets/images/thumbs/Voron1.jpg
    caption: "Mostly assembled gantry system"
  - type: image
    src: /assets/images/Voron2.jpg
    thumb: /assets/images/thumbs/Voron2.jpg
    caption: "Custom end-effector mount printed on the Markforge Mk2 with continuous carbon fiber reinforcement for stiffness"
  - type: image
    src: /assets/images/Voron5.jpg
    thumb: /assets/images/thumbs/Voron5.jpg
    caption: "Assembled end effector using a BondTech LGX and Mosquito Magnum extruder"
  - type: image
    src: /assets/images/Voron7.jpg
    thumb: /assets/images/thumbs/Voron7.jpg
    caption: "Mounted end-effector"
  - type: image
    src: /assets/images/Voron8.jpg
    thumb: /assets/images/thumbs/Voron8.jpg
    caption: "Wiring for the machine"

{% endproject %}

<!-- Wearable 3D Designs -->
{% project %}
name: Wearable3D

summary: "Wearable 3D Printed Designs with Integrated Electronics"

links:
  - name: Github
    url: https://github.com/coelacant1/Protogen-3D-Prints

dates: "2020 - Now"

contributors: "Solo Project"

description: |-
  Designed and built 3D-printable cosplay hardware-currently have six designs in total. For the Beta, Delta, Epsilon, and Sigma, each model is available on GitHub as STL files optimized for common 3D printers. Every project includes an electronics wiring guide, custom ProtoTracer firmware binaries, and source code.

  The Alpha and Gamma prototypes use real-time stereoscopic passthrough vision with a custom lens stack and display module. The vision-systems use modified software (running on a Raspberry Pi CM4) to reduce latency to about 15 ms and support the stereoscopic live synchronized display. This leaves as much area as possible for the LED displays. Each uses two synchronized microcontrollers to render live 3D graphics on the full-face displays. Integrated front-facing distance sensors and a 9-axis IMU modify the animation pipeline, so animations react instantly to motion and proximity.

slides:
  - type: video
    video_id: dfjvdJtsRFg
    caption: "Full-color RGB test of the display system - Gamma design"
  - type: video
    video_id: CoMETUpcQyA
    caption: "Internal stereoscopic vision system on the Gamma Design"
  - type: video
    video_id: cp83inwCs3M
    caption: "Display testing the vision system"
  - type: video
    video_id: ii_2rAg5L5c
    caption: "Test of an assembled Delta design"
  - type: video
    video_id: 8PgzrYZm6BE
    caption: "Test of an assembled Beta design"
  - type: video
    video_id: u2CADiNy3Wo
    caption: "Face display and close ups of the Gamma"
  - type: video
    video_id: h3FrI2zM1bI
    caption: "Power testing with 25% max brightness - full white will draw over 100A at 5V"
  - type: video
    video_id: bU4lSFczVpw
    caption: "First testing of the Alpha design with the RGB panels"
  - type: video
    video_id: xQfBLp0Ws1g
    caption: "Testing of the Alpha design"
  - type: image
    src: /assets/images/epsilon3.jpg
    thumb: /assets/images/thumbs/epsilon3.jpg
    caption: "Side view of the Epsilon design"
  - type: image
    src: /assets/images/epsilon2.jpg
    thumb: /assets/images/thumbs/epsilon2.jpg
    caption: "Side view of the assembled Epsilon"
  - type: image
    src: /assets/images/epsilon4.jpg
    thumb: /assets/images/thumbs/epsilon4.jpg
    caption: "Epsilon split apart for manufacturing"
  - type: image
    src: /assets/images/alpha.jpg
    thumb: /assets/images/thumbs/alpha.jpg
    caption: "Internal view of the Alpha design"
  - type: image
    src: /assets/images/alpha3.jpg
    thumb: /assets/images/thumbs/alpha3.jpg
    caption: "Close up internal view of the Alpha"
  - type: image
    src: /assets/images/alpha2.jpg
    thumb: /assets/images/thumbs/alpha2.jpg
    caption: "Alpha design without RGB panels"
  - type: image
    src: /assets/images/alpha4.jpg
    thumb: /assets/images/thumbs/alpha4.jpg
    caption: "Rear view of the Alpha design without the RGB panels"
  - type: image
    src: /assets/images/beta.jpg
    thumb: /assets/images/thumbs/beta.jpg
    caption: "Beta design"
  - type: image
    src: /assets/images/beta3.jpg
    thumb: /assets/images/thumbs/beta3.jpg
    caption: "Beta design with rear door open"
  - type: image
    src: /assets/images/delta1.jpg
    thumb: /assets/images/thumbs/delta1.jpg
    caption: "Delta design"
  - type: image
    src: /assets/images/delta3.jpg
    thumb: /assets/images/thumbs/delta3.jpg
    caption: "Delta design rear view with the door open"
  - type: image
    src: /assets/images/delta2.jpg
    thumb: /assets/images/thumbs/delta2.jpg
    caption: "Transparent rendering of the Delta to show the screw mounting for assembly"

{% endproject %}

<!-- Dual Tilt Quadcopter -->
{% project %}
name: DTQuadcopter

summary: "Dual-Tilt Rotor Quadcopter"

dates: "2018 - 2019"

contributors: "Software Solo | Hardware by Co-Workers"

links:
  - name: GitHub
    url: https://github.com/coelacant1/Dual-Tilt-Rotor-Quadcopter

description: |-
  This project is an integrated control system for a quadcopter with independently tilting rotors. Each motor can pivot on two axes, letting the quadcopter perform complex maneuvers and fly at any angle. This was designed as a system to allow the quadcopter act as a gimbal to allow for a larger statically mounted camera while having a full 360 degree view.

  Software Stack:
  - Control algorithms in C++ for real-time dynamics
  - C#-based physics simulation with a C++/CLI bridge and control-structure library for tilt-rotor modeling
  Hardware Integration:
  - Runs on a Raspberry Pi for onboard flight control
  - Uses a PCA9685 16-channel PWM driver to manage motor speed and tilt servos
  - TCA9548A I2C multiplexer with multiple IMUs (MPU6050/MPU9150) for attitude sensing
  Highlights:
  - Custom pipelines for planning aerial maneuvers and stabilizing flight
  - Sensor fusion and control loops for autonomous operation

slides:
  - type: video
    video_id: q7M9ar37VO4
    caption: "Example of the control system simulation built in C++ with a C# front-end, this test found a few extremes which prevented consistent motion"
  - type: video
    video_id: Q-hpOqeRDjg
    caption: "Older example of the simulation which had several additional failures that caused unpredictable paths"

{% endproject %}

<!-- Other Projects-->
{% project %}
name: OtherProjects

summary: "Other Product Prototyping, Engineering, and Software"

links:
  - name: Github
    url: https://github.com/coelacant1/Protogen-3D-Prints

description: |-
  - Electrospinner: Designed and built an automated electrospinning device for researching polymer nanofibers (With Co-Worker)
  - Custom Print Farm: Designed and built custom compact and portable CoreXY stackable 3D printer farm (With Co-Workers)
  - Tripteron 3D Printer: Developed a Cartesian-parallel 3D printer prototype
  - Reflow Heater Controller: Created hardware and firmware for a reflow heater, featuring PID temperature control and programmable profiles
  - Printer Conversion for PEEK/Ultem: Modified a Stratasys FDM printer's hot end, control electronics, and build chamber to print high-temperature materials (PEEK/Ultem/Polycarbonate)
  - Ozone Water Treatment System: Created a residential ozone generator for point-of-entry water sanitation, including ozone injection and feedback control with dissolved-ozone sensors
  - Environment Monitoring System: Built a networked sensor station (BME280 + ESP8266) for temperature, humidity, and pressure monitoring, with data upload to a cloud dashboard and low-power deep-sleep operation
  - Face Shield Production (COVID, 6,000+ units): Organized and scaled PETG face-shield fabrication using laser-cut visors, injection-molded frames, and die-cut elastic (with co-worker)
  - Battlebot: Designed and constructed a 3 lb combat robot featuring high-torque motors, LiPo power management, an all-wheel drivetrain optimized for speed and acceleration, and custom armor (with co-worker)
  - High-Speed USB Current Device: Created a portable measurement device capable of sampling voltage and current at kilohertz rates-ideal for reverse-engineering ongoing tasks on electronics
  - Medical Mattress Squeeze Table: Built a pneumatic squeeze-table assembly used in medical mattress manufacturing (with Co-Worker)
  - Medical Mattress/Pad Heater: Built custom heating apparatus for sealing seams on medical mattress pads (with Co-Worker)
  - Window Clip Manufacturing: Recreated obsolete window-clip parts via bulk additive manufacturing to replace thousands of failed components (with co-worker)
  - Multi-Color FDM Production: Designed prototype parts for a consumer product and managed printing and post-processing of 500 multi-color FDM components
  - Custom Drives Hardware: Designed and built drives electronics for three-phase AC motors, inductive motors, and DC motors in a single enclosure
  - Topographic Map: Designed and created 12'x5' topographic map for education and research center (with Co-Worker)

slides:
  - type: image
    src: /assets/images/CAD26.jpg
    thumb: /assets/images/thumbs/CAD26.jpg
    caption: "Render of the stroboscope enclosure"
  - type: image
    src: /assets/images/CAD23.jpg
    thumb: /assets/images/thumbs/CAD23.jpg
    caption: "View of the internals of the stroboscope enclosure"
  - type: image
    src: /assets/images/Electronics3.jpg
    thumb: /assets/images/thumbs/Electronics3.jpg
    caption: "Electronics of the stroboscope"
  - type: image
    src: /assets/images/Electronics7.jpg
    thumb: /assets/images/thumbs/Electronics7.jpg
    caption: "Electronics and test fit in the stroboscope enclsoure"
  - type: image
    src: /assets/images/AdvancedAdditive1.jpg
    thumb: /assets/images/thumbs/AdvancedAdditive1.jpg
    caption: "Copper DMLS metal prints with integrated cooling channels"
  - type: image
    src: /assets/images/CAD20.jpg
    thumb: /assets/images/thumbs/CAD20.jpg
    caption: "CAD for the Drives Hardware"
  - type: image
    src: /assets/images/Electronics9.jpg
    thumb: /assets/images/thumbs/Electronics9.jpg
    caption: "LoRa based remote firework controller"
  - type: image
    src: /assets/images/Electronics29.jpg
    thumb: /assets/images/thumbs/Electronics29.jpg
    caption: "Controllers for LED driving"
  - type: image
    src: /assets/images/Electronics32.jpg
    thumb: /assets/images/thumbs/Electronics32.jpg
    caption: "Environmental temperature monitors"
  - type: image
    src: /assets/images/Electronics27.jpg
    thumb: /assets/images/thumbs/Electronics27.jpg
    caption: "End Effector Controller for the high speed pick and place"
  - type: image
    src: /assets/images/PolyJet1.jpg
    thumb: /assets/images/thumbs/PolyJet1.jpg
    caption: "Multi-material wheel printed on a polyjet for a robot platform"
  - type: image
    src: /assets/images/CAD3.jpg
    thumb: /assets/images/thumbs/CAD3.jpg
    caption: "CAD for custom polishing device"
  - type: image
    src: /assets/images/Electronics6.jpg
    thumb: /assets/images/thumbs/Electronics6.jpg
    caption: "Gallium Nitride based 5V 20A Step-Down Converter"
  - type: image
    src: /assets/images/Electronics24.jpg
    thumb: /assets/images/thumbs/Electronics24.jpg
    caption: "LED Controllers after pick-and-placing and reflow"
  - type: pdf
    src: /assets/pdfs/KS35_ElectronicKitGuide_V1-0.pdf
    caption: "Guide for the open-source electronics for the 8-Channel LED Controller"

{% endproject %}

## **Advanced Manufacturing**
<!-- Advanced Manufacturing Technologies -->
{% project %}
name: AdditiveManufacturing

summary: "Advanced Additive Manufacturing Technologies"

dates: "2022 - Now"

contributors: "Collaboration with Co-Worker"

description: |-
  Operator and technician for additive manufacturing platforms. Sharing responsibility for hands‑on testing, day‑to‑day operation, and performing preventative maintenance and repairs.
  
  Key systems managed:
  - Xact XM200G (DMLS) - Metal powder handling, build chamber calibration, and print configuration/support creation
  - Stratasys Objet 260 (PolyJet) - Multi‑material jetting setup, print head maintenance, and resin reservoir management
  - Sinterit Lisa and Lisa X (Nylon SLS) - Powder bed preparation, laser power calibration, part depowdering workflows, and periodic chamber cleaning
  - ProJet 660 Pro (Binder Jetting) - Binder delivery system checks, powder/agent replenishment, print tray realignment, and part post processing
  - Markforged Mark 2 (Composite) - Print parameter tuning, fiber reinforcement strategies, and routine nozzle and drive‑system upkeep
  - 23X BambuLab P1S FDM
  - 4X BambuLab A1 FDM
  - 1X BambuLab X1C FDM
  - 2X BambuLab H2D FDM
  - 2X Prusa XL Multi-Toolhead FDM
  - 2X Prusa i3 MK4 FDM
  - 16X Prusa i3 MK3s FDM
  - 10X Prusa Mini FDM
  - Formlabs Form 4 Resin
  - Other miscellaneous printers (~75 total)

slides:
  - type: video
    video_id: uqiq7QweEDk
    caption: "Direct-Metal Laser Sintering on an XACT XM200G"
  - type: video
    video_id: fdjnXAUBCGc
    caption: "Selective Laser Sintering on a Sinterit Lisa X, printing parts in Nylon"
  - type: video
    video_id: dFCbU9youog
    caption: "Selective Laser Sintering on a Sinterit Lisa, printing parts in Nylon"
  - type: video
    video_id: BPy7IkN17Vo
    caption: "Stratasys Objet 260 PolyJet printing a flexible mold with disolvable support"
  - type: video
    video_id: pM5lK7U_KQc
    caption: "ProJet 660 Pro BinderJet 3D printer printing multi-color designs"
  - type: video
    video_id: uj5rvHp6Iho
    caption: "Post-Processing the BinderJet prints"
  - type: image
    src: /assets/images/Manufacturing2.jpg
    thumb: /assets/images/thumbs/Manufacturing2.jpg
    caption: "A 3D printed Scooby-Doo prior to cleaning"
  - type: image
    src: /assets/images/Manufacturing4.jpg
    thumb: /assets/images/thumbs/Manufacturing4.jpg
    caption: "Scooby-Doo after cleaning the print"
  - type: image
    src: /assets/images/Manufacturing10.jpg
    thumb: /assets/images/thumbs/Manufacturing10.jpg
    caption: "Bust for a client printed in 316 stainless steel"
  - type: image
    src: /assets/images/Manufacturing11.jpg
    thumb: /assets/images/thumbs/Manufacturing11.jpg
    caption: "Close up of the bust printed for a client"
  - type: image
    src: /assets/images/AdvancedAdditive1.jpg
    thumb: /assets/images/thumbs/AdvancedAdditive1.jpg
    caption: "Copper cooling channels printed on the XACT XM200G"
  - type: image
    src: /assets/images/AdvancedAdditive3.jpg
    thumb: /assets/images/thumbs/AdvancedAdditive3.jpg
    caption: "CoreXY 3D printer end-effector frame with support structure in Markforged Eiger"
  - type: image
    src: /assets/images/AdvancedAdditive9.jpg
    thumb: /assets/images/thumbs/AdvancedAdditive9.jpg
    caption: "Internal structure of the continuous carbon-fiber reinforcement on the Markforged print"
  - type: image
    src: /assets/images/AdvancedAdditive6.jpg
    thumb: /assets/images/thumbs/AdvancedAdditive6.jpg
    caption: "Test prints off of the Stratasys PolyJet"
  - type: image
    src: /assets/images/PolyJet2.jpg
    thumb: /assets/images/thumbs/PolyJet2.jpg
    caption: "Flexible structure printed on the PolyJet"
  - type: image
    src: /assets/images/PolyJet1.jpg
    thumb: /assets/images/thumbs/PolyJet1.jpg
    caption: "Mixed-durometer materials with rubber and ABS-like resin on the PolyJet for product testing"
  - type: image
    src: /assets/images/BinderJet1.jpg
    thumb: /assets/images/thumbs/BinderJet1.jpg
    caption: "Full-color test on the ProJet 660 Pro BinderJet"

{% endproject %}


<!-- Pick and Place Line -->
{% project %}
name: PicknPlaceLine

summary: "Installation and Operation of Automated Pick-and-Place Assembly Line"

dates: "2019 - 2021"

contributors: "Solo Project | Assistance with Moving Equipment"

description: |-
  Led the installation, configuration, and operation of an automated pick-and-place assembly line, overseeing the import of the equipment and integrating it within our facility. This project required infrastructure planning, and teardown/rebuild of machinery to match our requirements.

  The final setup included:
  - Semi-Automatic Solder Paste Screen Printer (KAYO-5088)
  - Manual Screen Printer (KAYO-P1000)
  - Solder Paste Mixer (KAYO-500S)
  - Automatic PCB Unloader (KAYO-50S)
  - 2X Inspection Conveyors (KAYO-ICO6)
  - Automatic Pick-and-Place Machine (KAYO-A4L)
  - Convection Reflow Oven (KAYO-RF430)
  - Automatic PCB Loader (KAYO-50P)
  - Post Inspection Station (Digital Microscope/Testing Hardware)

slides:
  - type: image
    src: /assets/images/AssemblyLine6.jpg
    thumb: /assets/images/thumbs/AssemblyLine6.jpg
    caption: "Final implementation of a Kayo SMT assembly line"
  - type: image
    src: /assets/images/AssemblyLine1.jpg
    thumb: /assets/images/thumbs/AssemblyLine1.jpg
    caption: "Process of assembly for the Kayo Pick n Place"
  - type: image
    src: /assets/images/AssemblyLine3.jpg
    thumb: /assets/images/thumbs/AssemblyLine3.jpg
    caption: "Assembled frame for the Pick n Place"
  - type: image
    src: /assets/images/AssemblyLine4.jpg
    thumb: /assets/images/thumbs/AssemblyLine4.jpg
    caption: "Pick n Place assembled"
  - type: image
    src: /assets/images/AssemblyLine7.jpg
    thumb: /assets/images/thumbs/AssemblyLine7.jpg
    caption: "Internal view of the pick n place"
  - type: video
    video_id: dFcBl_joEPg
    caption: "Testing the first head of the pick n place"
  - type: video
    video_id: o89Dq8lxETg
    caption: "Second Test of the pick n place head"

{% endproject %}

<!-- Large Scale 3d Printing -->
{% project %}
name: LargePrinter

summary: "Design and Creation of Large-Scale Custom Printers"

dates: "2015 - Now"

contributors: "Design/Software | Project with Two Co-Workers"

description: |-
  Collaborated with two engineers to build a large-format Delta 3D printer prototype (10ft tall x 6ft wide) in 45 days. Although intended as a proof of concept, it produced usable parts up to 6ft tall.
  
  Contributions:
  - Kinematic Calibration: Leveraged OpenDACT to compute all machine kinematic parameters (arm lengths, endstop offsets, carriage geometry) for a 4 ft‑diameter bed
  - Prototype Mechanical Design: Co‑designed structural linkages, custom brackets, and frame supports; specified and validated actuators and carriage assemblies
  - Motion and Slicer Tuning: Developed and refined slicing profiles and motion parameters (speeds, accelerations, jerk limits)
  - Delivery: Helped all stages-from concept through calibration and test prints-within a 45‑day schedule, and ended with a production‑capable prototype

slides:
  - type: image
    src: /assets/images/10ftDelta.jpg
    thumb: /assets/images/thumbs/10ftDelta.jpg
    caption: "10ft tall delta style 3D printer laying on it's side - had to be moved to a separate room to go vertical"
  - type: image
    src: /assets/images/10ftDeltaTest.jpg
    thumb: /assets/images/thumbs/10ftDeltaTest.jpg
    caption: "First test print of the large-format delta 3D printer"

{% endproject %}

<!-- Print Farm Implementation and Management -->
{% project %}
name: PrintFarm

summary: "Print Farm Implementation and Management"

dates: "2023 - Now"

contributors: "Implemented with Co-Worker"

description: |-
  Worked with a co-worker to implement and operate a 30x BambuLab (P1S, X1 Series X1C and H2D, A1/A1 Mini) and 30x Prusa (MK4S, XL, MK3S, Mini) 3D printer farm, I helped optimize most aspects of our additive manufacturing workflow:
  - Deployment and Configuration: Deployed and networked 30 BambuLab machines, standardizing slicer settings, and filament profiles to ensure consistent performance across P1S, X1C, H2D, A1, and A1 Minis. As well as an additional 30 Prusa machines, some offline and some managed by PrusaConnect
  - Process Optimization: Tuned print profiles (temperatures, speeds, etc) for various materials-PLA, PETG, ASA, and composites
  - Preventive Maintenance: Managed maintenance routines (lubrication, nozzle swaps)
  - Quality Control and Troubleshooting: Diagnosed and resolved failures, and optimized processes for batch print jobs

slides:
  - type: image
    src: /assets/images/PrintFarm1.jpg
    thumb: /assets/images/thumbs/PrintFarm1.jpg
    caption: "Bambu P1S Print Farm set up with back up UPS'"
  - type: image
    src: /assets/images/PrintFarm2.jpg
    thumb: /assets/images/thumbs/PrintFarm2.jpg
    caption: "Partially finished unboxing and validating printers"
  - type: image
    src: /assets/images/PrintFarm5.jpg
    thumb: /assets/images/thumbs/PrintFarm5.jpg
    caption: "Print farm still in the boxes"

{% endproject %}

<!-- Custom Tooling -->
{% project %}
name: CustomTooling

summary: "Injection Molding, Vacuum Forming, and Specialized Tooling Design"

dates: "2011 - Now"

contributors: "Face Shields with Co-Worker | Rest are Solo Projects"

description: |-
  Developed tooling solutions for injection molding, vacuum forming, and jig creation for repeated tasks:
  - Injection‑Molded Face Shield Frames: Designed parts optimized for automated injection molding; tooling produced over 6,000 units for local clinics and hospitals
  - Custom Vacuum‑Forming Bucks: Engineered master forms (bucks) for thermoforming complex geometries
  - Visor‑Cutting Jigs: Created fixtures to hold and index vacuum-formed visors for manual and automated trimming
  - PCB Stenciling Fixtures: Developed alignment jigs that secure PCBs for solder‑paste application
  - Custom Production Centering Tools: Designed and fabricated fixtures for CO2 laser and fiber laser etching

slides:
  - type: image
    src: /assets/images/FaceShields0.jpg
    thumb: /assets/images/thumbs/FaceShields0.jpg
    caption: "Final STEP design sent to a local manufacturer to EDM the mold and begin production on ~6000 face shields"
  - type: image
    src: /assets/images/FaceShields3.jpg
    thumb: /assets/images/thumbs/FaceShields3.jpg
    caption: "Test 3D print of the face shield head band"
  - type: image
    src: /assets/images/FaceShields2.jpg
    thumb: /assets/images/thumbs/FaceShields2.jpg
    caption: "Test 3D print of the face shield head band"
  - type: image
    src: /assets/images/FaceShields1.jpg
    thumb: /assets/images/thumbs/FaceShields1.jpg
    caption: "Test fit with the laser cut PETG face covering"
  - type: image
    src: /assets/images/FaceShields6.jpg
    thumb: /assets/images/thumbs/FaceShields6.jpg
    caption: "Partial pickup of the injection molded head bands"
  - type: image
    src: /assets/images/FaceShields11.jpg
    thumb: /assets/images/thumbs/FaceShields11.jpg
    caption: "Custom die-cut rubber head straps"
  - type: image
    src: /assets/images/FaceShields8.jpg
    thumb: /assets/images/thumbs/FaceShields8.jpg
    caption: "Assembly of the face shields"
  - type: image
    src: /assets/images/FaceShields5.jpg
    thumb: /assets/images/thumbs/FaceShields5.jpg
    caption: "Packaging the face shields"
  - type: image
    src: /assets/images/epsilon4.jpg
    thumb: /assets/images/thumbs/epsilon4.jpg
    caption: "3D print optimized for parallel 3D printing "
  - type: image
    src: /assets/images/Beta7.jpg
    thumb: /assets/images/thumbs/Beta7.jpg
    caption: "3D print optimized for printing and assembly"
  - type: image
    src: /assets/images/Beta9.jpg
    thumb: /assets/images/thumbs/Beta9.jpg
    caption: "Another 3D print optimized for printing and assembly"
  - type: image
    src: /assets/images/Beta6.jpg
    thumb: /assets/images/thumbs/Beta6.jpg
    caption: "Batch of parts printed off several machines"
  - type: image
    src: /assets/images/CAD7.jpg
    thumb: /assets/images/thumbs/CAD7.jpg
    caption: "Part optimized for 3D printing"
  - type: image
    src: /assets/images/delta2.jpg
    thumb: /assets/images/thumbs/delta2.jpg
    caption: "Transparent diagram showing how the Delta is assembled"
  - type: image
    src: /assets/images/Visors3.jpg
    thumb: /assets/images/thumbs/Visors3.jpg
    caption: "Vacuum forming buck just finished on the 3D printer"
  - type: image
    src: /assets/images/Visors2.jpg
    thumb: /assets/images/thumbs/Visors2.jpg
    caption: "Unsanded buck and sanded buck for two separate designs"
  - type: image
    src: /assets/images/Visors4.jpg
    thumb: /assets/images/thumbs/Visors4.jpg
    caption: "First test-form, had to adjust the angle and temperature on the former (reduce the webbing)"
  - type: image
    src: /assets/images/Visors1.jpg
    thumb: /assets/images/thumbs/Visors1.jpg
    caption: "Cutting guide for the visor, cut around the outside for the final shape"
  - type: image
    src: /assets/images/Tools1.jpg
    thumb: /assets/images/thumbs/Tools1.jpg
    caption: "Jig for holding PCBs for stenciling solder-paste"
  - type: image
    src: /assets/images/Tools2.jpg
    thumb: /assets/images/thumbs/Tools2.jpg
    caption: "Alternate view of the jig"
  - type: image
    src: /assets/images/Tools3.jpg
    thumb: /assets/images/thumbs/Tools3.jpg
    caption: "Custom printed jigs for holding glasses in a laser-cutter"
  - type: image
    src: /assets/images/Tools4.jpg
    thumb: /assets/images/thumbs/Tools4.jpg
    caption: "Close-up of the glass jig"
  - type: image
    src: /assets/images/4Axis3.jpg
    thumb: /assets/images/thumbs/4Axis3.jpg
    caption: "Custom tool for cutting vacuum formed visors"
  - type: image
    src: /assets/images/vacuumformer4.jpg
    thumb: /assets/images/thumbs/vacuumformer4.jpg
    caption: "Cutting guide for a vaccuum formed visor and frame for the custom vacuum forming table"
  - type: image
    src: /assets/images/VacuumFormer5.jpg
    thumb: /assets/images/thumbs/VacuumFormer5.jpg
    caption: "Visor after the dyeing process and installing magnets"
  - type: image
    src: /assets/images/vacuumformer3.jpg
    thumb: /assets/images/thumbs/vacuumformer3.jpg
    caption: "Visor before the dyeing process"
  - type: image
    src: /assets/images/vacuumformer2.jpg
    thumb: /assets/images/thumbs/vacuumformer2.jpg
    caption: "Dyed vacuum formed visor "
  - type: image
    src: /assets/images/CAD19.jpg
    thumb: /assets/images/thumbs/CAD19.jpg
    caption: "Static cutting guide for a formed visor"
  - type: image
    src: /assets/images/CAD3.jpg
    thumb: /assets/images/thumbs/CAD3.jpg
    caption: "Custom polisher for polishing 3D printed resin parts"

{% endproject %}

<!-- Other Manufacturing Expertise -->
{% project %}
name: OtherManufacturing

summary: "Other Manufacturing Technology Expertise"

dates: "2020 - Now"

contributors: "Share with Co-Worker"

description: |-
  Operator, programmer, and technician for a variety of manufacturing tools:
  - 2X 3/4‑Axis CNC Milling with a Tormach 1100MX and 1X Tormach 440MX
  - Engraving and cutting with BossLaser Fiber (1X) and CO₂ Lasers (2X) + Rotary
  - CNC plasma cutting with a Tormach 1300PL
  - Waterjet cutting with an Omax 1508
  - 3D scanning with the Creaform Go!SCAN Spark and VXElements software suite

slides:
  - type: video
    video_id: c9JMAAcL7XQ
    caption: "Cutting brass on a waterjet"
  - type: video
    video_id: 5qvgw3AXIjw
    caption: "Laser etching brass"
  - type: video
    video_id: fTPXXytuasA
    caption: "Cutting custom foam panels for a headset"
  - type: video
    video_id: 62ABiiJnl1c
    caption: "Installed and configured two Tormach 1100MX machines with tool changers"
  - type: video
    video_id: OV2HV-9dwZw
    caption: "Plasma cutting 3/8 mild steel plates"
  - type: image
    src: /assets/images/Manufacturing5.jpg
    thumb: /assets/images/thumbs/Manufacturing5.jpg
    caption: "Test cut off of the Tormach 1100MX"
  - type: image
    src: /assets/images/Manufacturing9.jpg
    thumb: /assets/images/thumbs/Manufacturing9.jpg
    caption: "Second test cut off the Tormach 1100MX"
  - type: image
    src: /assets/images/CO2Laser.jpg
    thumb: /assets/images/thumbs/CO2Laser.jpg
    caption: "Laser cut EVA foam"
  - type: image
    src: /assets/images/vacuumformer3.jpg
    thumb: /assets/images/thumbs/vacuumformer3.jpg
    caption: "Vacuum forming with a Formech former"
  - type: image
    src: /assets/images/Stand1.jpg
    thumb: /assets/images/thumbs/Stand1.jpg
    caption: "Custom metal fabrication - plasma cut plates, manual metal working/bending, welding, and powder coating"
  - type: image
    src: /assets/images/Stand5.jpg
    thumb: /assets/images/thumbs/Stand5.jpg
    caption: "Stand prior to powder-coating"
  - type: image
    src: /assets/images/Stand3.jpg
    thumb: /assets/images/thumbs/Stand3.jpg
    caption: "Post-baking in the powder-coating oven"
  - type: image
    src: /assets/images/Scan1.jpg
    thumb: /assets/images/thumbs/Scan1.jpg
    caption: "WIP laser scanning example"
  - type: image
    src: /assets/images/Scan2.jpg
    thumb: /assets/images/thumbs/Scan2.jpg
    caption: "WIP laser scanning example"

{% endproject %}

## **High-Performance Computing and Systems Infrastructure**
<!-- Hyper-Converged Infrastructure -->
{% project %}
name: Server Infrastructure

summary: "Hyper-Converged Infrastructure (HCI) Design and Implementation"

dates: "2019 - Now"

contributors: "Solo Implementation, Management, and Operation"

description: |-
  Designed and implemented 20-node hyper-converged ProxmoxVE cluster, concurrently hosting up to 3,000-4,000 virtual machines and containers:

  Specifications for this infrastructure:
  - Scalable Compute Resources:
    - Aggregate Capacity: 1,464 CPU cores and 10.8TiB RAM
    - Per-Node: Up to 80 CPU cores and 784GiB RAM
    - Accelerators: Select nodes equipped with NVIDIA RTX 8000 GPUs for GPU-accelerated tasks
    - Customization: Users can request private compute infrastructures of up to 20-nodes, with Slurm-ready configurations
  - Tiered Storage Solutions:
    - Ceph-backed dynamic scaling across three performance tiers:
      - SSD Base
      - SSD Low-Latency
      - HDD Base
    - Total Capacity: ~211.52TiB with automated data distribution and resilience
  - High-Speed Networking:
    - Bandwidth: Two 25 GbE links per server (50 GbE aggregate) configured with 802.3ad LACP and MC-LAG for redundancy
    - Features:  RDMA, SDN, jumbo frames, and traffic prioritization
  - Customization and security:
    - Full user control over the software stack and container/VM images
    - Software-defined networking with per-instance firewall rules
    - Support for external network hosting and user-deployed firewalls
    - Customized firewall rules per virtual instance depending on requirements
  - Reliability and availability:
    - Live scaling to adapt to workload demands without downtime
    - Redundant network paths and daily backup/snapshot schedules
    - Designed for security testing, scientific simulations, and parallel computing for data analysis and GPU-intensive applications
    - Preconfigured templates for nested multi-machine clusters with SLURM

slides:
  - type: image
    src: /assets/images/ServersInf1.jpg
    thumb: /assets/images/thumbs/ServersInf1.jpg
    caption: "Server and networking rack configuration, center is the primary 20-node cluster"
  - type: image
    src: /assets/images/Servers18.jpg
    thumb: /assets/images/thumbs/Servers18.jpg
    caption: "Current cluster and network layout"
  - type: image
    src: /assets/images/ServersInf5.jpg
    thumb: /assets/images/thumbs/ServersInf5.jpg
    caption: "Cluster resources from ProxmoxVE UI"
  - type: image
    src: /assets/images/ServersInf3.jpg
    thumb: /assets/images/thumbs/ServersInf3.jpg
    caption: "Ceph storage cluster status"
  - type: image
    src: /assets/images/ServersInf2.jpg
    thumb: /assets/images/thumbs/ServersInf2.jpg
    caption: "Ceph cluster storage configuration"
  - type: image
    src: /assets/images/Servers17.jpg
    thumb: /assets/images/thumbs/Servers17.jpg
    caption: "Secondary rack install"
  - type: image
    src: /assets/images/Servers13.jpg
    thumb: /assets/images/thumbs/Servers13.jpg
    caption: "Secondary rack install"
  - type: image
    src: /assets/images/Servers12.jpg
    thumb: /assets/images/thumbs/Servers12.jpg
    caption: "Layout of secondary rack"
  - type: image
    src: /assets/images/Servers11.jpg
    thumb: /assets/images/thumbs/Servers11.jpg
    caption: "Secondary rack wiring"
  - type: image
    src: /assets/images/Servers2.jpg
    thumb: /assets/images/thumbs/Servers2.jpg
    caption: "Intermediate setup of the secondary rack"
  - type: image
    src: /assets/images/Servers1.jpg
    thumb: /assets/images/thumbs/Servers1.jpg
    caption: "Configuration of the primary rack"
  - type: image
    src: /assets/images/Servers3.jpg
    thumb: /assets/images/thumbs/Servers3.jpg
    caption: "Initial state of network patch wiring"
  - type: image
    src: /assets/images/Servers4.jpg
    thumb: /assets/images/thumbs/Servers4.jpg
    caption: "After first rewire of network patches"
  - type: image
    src: /assets/images/PVELoadBalancer3.jpg
    thumb: /assets/images/thumbs/PVELoadBalancer3.jpg
    caption: "Automated load balancer for the cluster - background service"
  - type: image
    src: /assets/images/ProxmoxScripts5.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts5.jpg
    caption: "Custom script library is used to manage and automate cluster tasks"
  - type: image
    src: /assets/images/ProxmoxScripts1.jpg
    thumb: /assets/images/thumbs/ProxmoxScripts1.jpg
    caption: "Example output of the cluster automation"

{% endproject %}

<!-- Networking and Security -->
{% project %}
name: NetworkingSecurity

summary: "Networking and Security"

dates: "2020 - Now"

contributors: "Solo Management and Operation"

description: |-
  Managed a variety of network configurations and their supporting software:

  - Firewall Configuration and Management
    - OPNSense / PFSense / SonicWall
      - Deployed in active-active and active-standby high-availability pairs
      - Application-aware filtering, IDS/IPS integration
      - Role-based administration with audit logging
      - Geo-IP blocking
  - Linux Networking Configuration and Management
    - VLAN trunking configuration, LACP setup, and network performance optimizations
    - Linux-based firewall configuration and management
    - OSPF routing configuration for small-scale clusters
    - Optimization for Ceph networked storage
  - VPN Configuration and Management
    - OpenVPN Hosting
      - Centralized server deployments for user and service tunnels
      - RADIUS/LDAP integration for authentication
    - Site-to-Site VPN Experience
      - OPNSense ↔ OPNSense, OPNSense ↔ Palo Alto, SonicWall ↔ Palo Alto
  - Networking Hardware Ecosystem
    - MC-LAG with ECS Aggregation Switches
      - Active/active link aggregation across top-of-rack and spine layers for redundancy
    - VLAN segmentation and access control lists for tenant isolation
    - Deployment of Wi-Fi 7 and Wi-Fi 6/6E networks
    - IP Security Cameras and NVR Integration
      - PoE/PoE+ power delivery
      - Centralized NVR for video recording
  - Software-Defined Networking for Cluster Overlays
    - Custom L2/L3 overlays
      - Proxmox SDN module enabling per-tenant virtual networks within the cluster
    - Performance Monitoring with NetFlow
      - Flow telemetry to identify congestion with traffic paths
  - Cloudflare Zero Trust
    - Email-Based Access Controls
      - Secure web application access using one-time email verification links
      - Granular policy enforcement per URL, with logging of all user sessions
    - Remote Workstation and Internal App Protection
      - Proxy published internal services (RDP, SSH, web UIs) through Zero Trust tunnels
      - Device posture checks via WARP client

slides:
  - type: image
    src: /assets/images/network.jpg
    thumb: /assets/images/thumbs/network.jpg
    caption: "Current Ubiquiti-based network setup tree, uses OPNSense L3 router, 150-200 physical devices, several thousand virtual devices/servers"
  - type: image
    src: /assets/images/ServersInf1.jpg
    thumb: /assets/images/thumbs/ServersInf1.jpg
    caption: "Current rack configuration"
  - type: image
    src: /assets/images/ServersInf3.jpg
    thumb: /assets/images/thumbs/ServersInf3.jpg
    caption: "Ceph status for primary hyper-converged cluster"
  - type: image
    src: /assets/images/ZeroTrust.jpg
    thumb: /assets/images/thumbs/ZeroTrust.jpg
    caption: "Example setup of the Zero Trust access control management"

{% endproject %}

<!-- Active Directory Management -->
{% project %}
name: ActiveDirectory

summary: "Active Directory Management"

dates: "2019 - Now"

contributors: "Solo Management and Operation"

description: |-
  Active Directory Management (500+ users)

  - Self-Service Password Management
    - Automated portals enable users to reset and change their own passwords securely
    - MFA using TOTP for identity verification
  - Recurring Self-Audits
    - Scheduled scans of user accounts and group memberships to detect inactive or orphaned objects
    - Automated reports on password-policy compliance and access entitlements
  - Scalability and Concurrency
    - Supports ~500 users without performance degradation
    - High-availability domain controllers and load-balanced authentication
  - AD-Backed Network Access
    - Wi-Fi Access: 802.1X authentication against AD for per-user connectivity
    - VPN Access: AD credential validation for remote-user tunnels
  - Group-Based Policy Management
    - Role and department-based group management and network VLAN assignments
    - Dynamic group memberships allow policies to follow users as they change roles
  - Group Provisioning
    - Integration with HR/authoritative systems to import new users and remove old users
    - AD group creation and membership updates via scripted exports

slides:
  - type: image
    src: /assets/images/AD1.jpg
    thumb: /assets/images/thumbs/AD1.jpg
    caption: "Screen-cap of the current state of the active directory, ~2500 users, ~500 enabled in-use accounts, ~2000 accounts disabled (users that may return)"

{% endproject %}

<!-- Tools and Platforms -->
## **Tools and Platforms**
- Electronics and CAD: Altium Designer, KiCad, Autodesk Fusion 360, MATLAB
- Server and Virtualization: Proxmox VE, Ceph, VMWare vSphere, Windows Server, Hyper-V, Ansible, PDQ Deploy
- Programming Languages: C/C++, Python, JavaScript, C# (WPF, Unity), Bash, PowerShell, Make
- Environments and Administration: Linux and Windows administration, embedded platforms (ARM, RISC), GitHub Automation
- Miscellaneous: Blender, Adobe Suite (Photoshop, Illustrator, After Effects, Substance Painter), GIMP/Inkscape

<!-- This Website-->
### **Information on coela.dev**
{% project %}
name: CoelaDev

summary: "Coela.dev Website"

dates: "2025 - Now"

links:
  - name: Github
    url: https://github.com/coelacant1/coela.dev

description: |-
  The site's backend uses Ruby and Jekyll to compile Markdown and Liquid templates into a static site on push. A GitHub CI workflow rebuilds and deploys the site on changes to the main branch. On the front end, an HTML5 Canvas script animates sine-wave effects, and CSS with JavaScript handles glitch-text animations for headings.

{% endproject %}
