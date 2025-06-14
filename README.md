# 🚀 Devlog 01: RoboAnalyzer-based Online Competition

**Author:** Technocraft27 (Vedant Khopade)

---

## 📌 Introduction

Welcome to the first devlog of my robotics and simulation journey using Python and RoboAnalyzer concepts. This guide documents the implementation and visualization of forward and inverse kinematics for 2R and 3R planar manipulators using Python and MATLAB-style plotting.

---

## 👋 About Me

```python
print("hello world, I am Vedant Khopade. I am interested in design and development")
```

**Output:**

```
hello world, I am Vedant Khopade. I am interested in design and development
```

---

## 🤖 What is a 2R Planar Manipulator?

* **2R**: Two revolute (rotational) joints
* **Planar**: Operates in the 2D XY-plane

### 📐 Link Configuration

* **Link 1**: Length = L1, angle = θ1
* **Link 2**: Length = L2, angle = θ2 (relative to Link 1)

### ✅ Forward Kinematics Equations

To calculate the end-effector position (bx, by):

```python
ax = L1 * cos(θ1)
ay = L1 * sin(θ1)

bx = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
by = L1 * sin(θ1) + L2 * sin(θ1 + θ2)
```

---

## 🧮 Python Program: Forward Kinematics of 2R Manipulator

```python
import numpy as np
import matplotlib.pyplot as plt

# Setup
x_axis_coordinate_x = [-10, 10]
x_axis_coordinate_y = [0, 0]
y_axis_coordinate_x = [0, 0]
y_axis_coordinate_y = [-10, 10]

# Link 1
theta_1 = np.radians(30)
a1 = 5
ax = a1 * np.cos(theta_1)
ay = a1 * np.sin(theta_1)

# Link 2
theta_2 = np.radians(45)
a2 = 8
bx = ax + a2 * np.cos(theta_1 + theta_2)
by = ay + a2 * np.sin(theta_1 + theta_2)

# Plotting
plt.plot(x_axis_coordinate_x, x_axis_coordinate_y, 'k')
plt.plot(y_axis_coordinate_x, y_axis_coordinate_y, 'k')
plt.plot([0, ax], [0, ay], 'bo-', label='Link 1', linewidth=3)
plt.plot([ax, bx], [ay, by], 'ro-', label='Link 2', linewidth=2)
plt.title("Forward Kinematics of 2R Planar Manipulator")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
```

---

## 📐 MATLAB-style Python Program: Forward Kinematics of 3R Manipulator

```python
# Setup
# Link 3 configuration
theta_3 = np.radians(30)
a3 = 3

# Calculate Link 3 endpoint
cx = bx + a3 * np.cos(theta_1 + theta_2 + theta_3)
cy = by + a3 * np.sin(theta_1 + theta_2 + theta_3)

# Plotting
plt.plot([bx, cx], [by, cy], 'go-', label='Link 3', linewidth=4)
plt.title("Forward Kinematics of 3R Planar Manipulator")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
```

---

## 🔁 Inverse Kinematics: 2R Planar Manipulator

### 🎯 Objective:

Given the end-effector position (x, y), find joint angles θ₁ and θ₂.

```python
# Link lengths
a1 = 10
a2 = 7

# Desired position
x = 12
y = 5

# Calculate θ2 using the Law of Cosines
cos_theta2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
if abs(cos_theta2) > 1:
    raise ValueError("The point is outside the reachable workspace")

theta2 = np.arccos(cos_theta2)
k1 = a1 + a2 * np.cos(theta2)
k2 = a2 * np.sin(theta2)
theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

# Joint positions
x1 = a1 * np.cos(theta1)
y1 = a1 * np.sin(theta1)
x2 = x1 + a2 * np.cos(theta1 + theta2)
y2 = y1 + a2 * np.sin(theta1 + theta2)

# Plot
plt.plot([0, x1, x2], [0, y1, y2], 'o-', linewidth=3)
plt.title("2R Planar Manipulator - Inverse Kinematics")
plt.axis('equal')
plt.grid(True)
plt.show()
```

**Output:**

```
Joint Angle θ1: -9.58°
Joint Angle θ2: 81.79°
```

---

## 🤖 What is a 3R Planar Manipulator?

* Three revolute joints
* Movement in the XY-plane
* Three links: a1, a2, a3
* End-effector defined by position (x, y) and orientation φ

---

## 🔁 Inverse Kinematics: 3R Planar Manipulator

```python
# Link lengths
a1 = 10
a2 = 8
a3 = 5

# Desired end-effector position and orientation
x = 15
y = 5
phi = np.radians(45)

# Wrist position
xw = x - a3 * np.cos(phi)
yw = y - a3 * np.sin(phi)

# θ2
cos_theta2 = (xw**2 + yw**2 - a1**2 - a2**2) / (2 * a1 * a2)
if abs(cos_theta2) > 1:
    raise ValueError("Unreachable point.")

theta2 = np.arccos(cos_theta2)
k1 = a1 + a2 * np.cos(theta2)
k2 = a2 * np.sin(theta2)
theta1 = np.arctan2(yw, xw) - np.arctan2(k2, k1)
theta3 = phi - theta1 - theta2

# Joint positions
x1 = a1 * np.cos(theta1)
y1 = a1 * np.sin(theta1)
x2 = x1 + a2 * np.cos(theta1 + theta2)
y2 = y1 + a2 * np.sin(theta1 + theta2)
x3 = x2 + a3 * np.cos(theta1 + theta2 + theta3)
y3 = y2 + a3 * np.sin(theta1 + theta2 + theta3)

# Plotting
plt.plot([0, x1, x2, x3], [0, y1, y2, y3], 'o-', linewidth=3)
plt.title("3R Planar Manipulator - Inverse Kinematics")
plt.grid(True)
plt.axis('equal')
plt.show()
```

**Output:**

```
θ1 = -35.53°
θ2 = 100.96°
θ3 = -20.43°
```

---

## 🏁 Conclusion

This devlog demonstrates the simulation of 2R and 3R manipulators with complete kinematic calculations and 2D visualizations. The concepts explored form a foundational part of robotic arm control in both academic research and industry applications.

Stay tuned for the next devlog!

---

**Created by:** Technocraft27
**Platform:** Python (NumPy + Matplotlib)
