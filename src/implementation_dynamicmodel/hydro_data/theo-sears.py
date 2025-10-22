import numpy as np
import matplotlib.pyplot as plt
from scipy.special import hankel2
from math import pi

plt.rcParams['text.usetex'] = True


# Define Theodorsen function
def theodorsen_function(k):
    """Theodorsen function C(k) = H1^(2)(k) / (H1^(2)(k) + i*H0^(2)(k)),
    where Hn^(2) are Hankel functions of the second kind."""
    H1 = hankel2(1, k)
    H0 = hankel2(0, k)
    return H1 / (H1 + 1j * H0)

# Define Sears function
def sears_function(k):
    """Sears function S(k) = 1 / (1 + (1j * H0^(2)(k) / H1^(2)(k))),
    where Hn^(2) are Hankel functions of the second kind."""
    H1 = hankel2(1, k)
    H0 = hankel2(0, k)
    return (2j/(pi*k))/(H1+1j*H0)

# Define Sears function
def sears_function(k):
    """Sears function S(k) = 1 / (1 + (1j * H0^(2)(k) / H1^(2)(k))),
    where Hn^(2) are Hankel functions of the second kind."""
    H1 = hankel2(1, k)
    H0 = hankel2(0, k)
    return (2j/(pi*k))/(H1+1j*H0)

def sears_function_LE(k):
    """Sears function S(k) = 1 / (1 + (1j * H0^(2)(k) / H1^(2)(k))),
    where Hn^(2) are Hankel functions of the second kind."""
    H1 = hankel2(1, k)
    H0 = hankel2(0, k)
    return (np.exp(-1j*k))*(2j/(pi*k))/(H1+1j*H0)

# Define reduced frequencies (k)
k_values = np.linspace(0.001, 10, 1000)  # Avoid k=0 due to singularity

# Compute Theodorsen and Sears functions
theodorsen_values = theodorsen_function(k_values)
sears_values = sears_function(k_values)
searsLE_values = sears_function_LE(k_values)

# Specific k-values to annotate
k_annotate = [0.01, 0.05]

theodorsen_annotate = theodorsen_function(np.array(k_annotate))
sears_annotate = sears_function(np.array(k_annotate))
searsLE_annotate = sears_function_LE(np.array(k_annotate))

# Plot in the complex plane
fig = plt.figure(figsize=(10, 6))
ax_cartesian = fig.add_subplot(111)

ax_cartesian.plot(theodorsen_values.real, theodorsen_values.imag, label="Theodorsen Function", color="#3b67ad")
ax_cartesian.plot(sears_values.real, sears_values.imag, label="Sears Function", color="#217319")


# Annotate the points for specific k-values
for i, k in enumerate(k_annotate):
    # Annotate Theodorsen function
    ax_cartesian.scatter(theodorsen_annotate[i].real, theodorsen_annotate[i].imag, color="#3b67ad", s=50)

    # Annotate Sears function
    ax_cartesian.scatter(sears_annotate[i].real, sears_annotate[i].imag, color="#217319", s=50, marker="s")


# Add labels and legend
ax_cartesian.axhline(0, color="black", linewidth=0.8, linestyle="--")
ax_cartesian.axvline(0, color="black", linewidth=0.8, linestyle="--")
plt.title("Theodorsen and Sears Functions in the Complex Plane")
ax_cartesian.set_xlabel("Real Part")
ax_cartesian.set_ylabel("Imaginary Part")
ax_cartesian.legend()
ax_cartesian.grid()
ax_cartesian.axis("equal")



plt.show()


# Convert to polar coordinates
theodorsen_magnitude = np.abs(theodorsen_values)
theodorsen_phase = np.angle(theodorsen_values)

sears_magnitude = np.abs(sears_values)
sears_phase = np.angle(sears_values)

searsLE_magnitude = np.abs(searsLE_values)
searsLE_phase = np.angle(searsLE_values)

# Plot in polar coordinates
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(12, 9))

# Sears plot
ax.plot(sears_phase, sears_magnitude, label="Sears", color="black", linewidth = 1)


# Theodorsen plot
ax.plot(theodorsen_phase, theodorsen_magnitude, label="Theodorsen", linestyle="dashed", color="black", linewidth = 1)


# Sears wrt LE plot
ax.plot(searsLE_phase, searsLE_magnitude, label="Modified Sears", linestyle="dashdot", color="black", linewidth = 1)


# Annotate the points for specific k-values
for i, k in enumerate(k_annotate):
    # Annotate Theodorsen function
    ax.scatter(np.angle(theodorsen_annotate[i]), np.abs(theodorsen_annotate[i]), color="black", s=10)

    # Annotate Sears function
    ax.scatter(np.angle(sears_annotate[i]), np.abs(sears_annotate[i]), color="black", s=10, marker="s")

    ax.scatter(np.angle(searsLE_annotate[i]), np.abs(searsLE_annotate[i]), color="black", s=10, marker="^")

ax.set_rlim(0.2,1)
ax.set_thetamin(-30)  # Set angular lower limit
ax.set_thetamax(20)  # Set angular upper limit
ax.set_aspect(0.5) 
ax.legend(loc="lower left")
ax.set_xticks(np.arange(-25*pi/180, 20*pi/180, 5*pi/180))

fig.savefig("theo-sears-polars.png", dpi=300)

plt.tight_layout()
plt.show()

