"""Parameters for the free-flyer additive manufacturing simulation.

Every value below is tagged with its provenance:

    [PAPER]    stated explicitly in Jonckers, Tauscher, Thakur & Maywald (2022),
               "Additive Manufacturing of Large Structures Using Free-Flying
               Satellites", Front. Space Technol. 3:879542,
               doi:10.3389/frspt.2022.879542
    [DERIVED]  computed from [PAPER] values
    [INFERRED] not given in the paper; chosen to be physically reasonable and,
               where noted, calibrated so that the simulation reproduces the
               error statistics the paper reports. These are the honest
               weak points of the recreation.

Units are SI internally (metres, seconds, kilograms, radians). Helpers ending
in ``_MM`` / ``_DEG`` are provided where the paper quotes those units, so the
numbers in this file can be read against the text directly.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Free-flyer rigid body
# ---------------------------------------------------------------------------

MASS_KG = 20.2
"""[PAPER] Sec. 2: three stacked modules "having a total weight of 20.2 kg"."""

INERTIA_ZZ = 0.539
"""[INFERRED] Yaw inertia about the body z axis, kg m^2.

The paper gives no inertia. Modelled as a uniform cuboid of side 0.4 m (the
free-flyer footprint implied by the 400 mm centre-to-TCP offset), so
I = m (w^2 + d^2) / 12 = 20.2 * (0.16 + 0.16) / 12 = 0.539 kg m^2.
"""

TCP_OFFSET_M = 0.400
"""[PAPER] Sec. 4.1: the TCP "is located approximately 400 mm from the
geometric centre of the free-flyer"."""

# ---------------------------------------------------------------------------
# Propulsion: 8 propeller thrusters
# ---------------------------------------------------------------------------

N_THRUSTERS = 8
"""[PAPER] Sec. 2: "eight propeller-based thrusters"."""

THRUSTER_ARM_M = 0.200
"""[INFERRED] Moment arm of each thruster pair from the body centre, m.
Consistent with the 0.4 m footprint assumed for INERTIA_ZZ."""

THRUSTER_MAX_N = 0.35
"""[INFERRED] Per-thruster saturation force, N.

Not stated. Chosen so the eight-thruster array can counter the worst-case
residual gravity (MASS_KG * RESIDUAL_GRAVITY_MAX = 0.194 N) with margin for
control authority, while remaining weak enough that the recovery from a
friction stick event takes the "some time" the paper describes (Sec. 5.1).
"""

THRUSTER_TAU_S = 0.20
"""[INFERRED] First-order thruster time constant, s.

The paper states the thrusters have "transfer functions corresponding to
first-order systems; however, they have longer rise and fall times to a step
input compared to cold gas thrusters" (Sec. 2) but gives no value. 0.2 s is a
representative propeller spin-up time and preserves the stated qualitative
behaviour (slower than a cold gas thruster, which is ~10-50 ms).
"""

THRUSTER_DEADBAND_N = 0.150
"""[INFERRED, CALIBRATED] Minimum commanded force that produces thrust, N.

Sec. 2: the demanded force "is then mapped linearly to the demanded rpm of the
thrusters and a pulse-width modulation (PWM) signal is generated accordingly
causing the propellers to spin". A propeller drive has an unavoidable startup
deadband -- below a threshold duty cycle the rotor does not spin fast enough to
generate useful thrust -- and this is the dominant reason a propeller-driven
free-flyer limit-cycles about its setpoint instead of settling quietly.

Calibrated so station keeping lands inside the paper's stated envelope of
"approximately +/- 2 mm in the x and y directions" and "+/- 0.5 deg" about z
(Sec. 4.1, Fig. 4). Without a deadband the simulated free-flyer holds position
an order of magnitude better than the real hardware, which would flatter the
uncompensated baseline and understate the value of the correction algorithm.
"""

PWM_LEVELS = 256
"""[INFERRED] PWM quantisation levels. Sec. 2 notes the payload microcontroller
is an Arduino Mega, whose default ``analogWrite`` PWM is 8-bit. Quantisation
coarsens the achievable wrench and contributes to the limit cycle."""

THRUSTER_SCALE = 2.0
"""[PAPER] Eq. 2: f = 2 M^dagger u. The factor of two compensates for the
thrusters being unidirectional (Zappulla et al., 2017)."""


def thruster_geometry() -> tuple[np.ndarray, np.ndarray]:
    """Return (positions, directions) of the 8 thrusters in the body frame.

    [INFERRED] The paper cites the Zappulla et al. (2017) layout but does not
    draw it. Modelled as the standard planar arrangement: one thruster pair at
    each of the four corners, the two members of a pair pushing along +/-x and
    +/-y, giving full 3-DoF (fx, fy, tau) authority with unidirectional
    thrusters. This reproduces the rank-3 mapping M required by Eq. 1.
    """
    a = THRUSTER_ARM_M
    positions = np.array(
        [
            [a, a], [a, a],
            [-a, a], [-a, a],
            [-a, -a], [-a, -a],
            [a, -a], [a, -a],
        ],
        dtype=float,
    )
    directions = np.array(
        [
            [-1.0, 0.0], [0.0, -1.0],
            [0.0, -1.0], [1.0, 0.0],
            [1.0, 0.0], [0.0, 1.0],
            [0.0, 1.0], [-1.0, 0.0],
        ],
        dtype=float,
    )
    return positions, directions


# ---------------------------------------------------------------------------
# Disturbances
# ---------------------------------------------------------------------------

RESIDUAL_GRAVITY_MAX = 9.6e-3
"""[PAPER] Sec. 4.1: residual gravitational acceleration "can reach
9.6 mm/s^2", caused by air bearing table panels not being perpendicular to
the gravity vector."""

PANEL_INCLINATION_VARIATION_DEG_PER_M = 0.05
"""[PAPER] Sec. 4.1: "each panel has a maximum inclination variation of
+/- 0.05 deg/m", and the residual acceleration "varies slowly across the
table"."""

AERO_FRACTION = 0.10
"""[PAPER] Sec. 4.1: aerodynamic forces/torques from the air blowing system
"account for less than 10% of all residual accelerations"."""

BEARING_FRICTION_COEFF = 1e-5
"""[PAPER] Sec. 4.1: the air bearing achieves "a friction coefficient of
< 10^-5"."""

TABLE_SIZE_M = (4.0, 7.0)
"""[PAPER] Sec. 2: "a 4 m x 7 m active air bearing table"."""

# ---------------------------------------------------------------------------
# Nozzle / substrate contact friction  (the calibrated parameter)
# ---------------------------------------------------------------------------

NOZZLE_FRICTION_N = 0.105
"""[INFERRED, CALIBRATED] Coulomb friction force between nozzle and substrate
while printing, N.

This is the one number the paper never quantifies, and the most important
caveat of this recreation. Sec. 5.1 identifies friction as the cause of the
uncompensated printing error ("This increase in error was observed to be
caused by friction between the nozzle and the print substrate, or already
printed material. Any applied force disturbs the position of the free-flyer,
which then requires some time to counteract the disturbance") and Sec. 5.3
notes that extruding below half the nozzle diameter "greatly increased
friction ... to the extent that the free-flyer would struggle to move", but no
coefficient or normal force is given.

It is therefore calibrated against the paper's *reported outcome*: the
uncompensated print mean TCP error of 8.42 mm (Sec. 5.1). The calibrated value
reproduces 8.70 mm. See tools/calibrate.py. It is NOT a paper value.
"""

NOZZLE_SNAG_N = 0.42
"""[INFERRED, CALIBRATED] Peak force of a localised high-friction "snag", N.

Sec. 5.1: "The point of maximum error was observed to occur at the same
position for repeated experiments, and was caused by particularly high
friction between the nozzle and already printed structure." Calibrated toward
the 55.45 mm uncompensated maximum error; the calibrated value reaches about
48 mm, i.e. the right order of magnitude and the right qualitative behaviour
(one localised excursion an order of magnitude above the mean) without matching
exactly. Deterministic in path position, matching the paper's observation of
repeatability.
"""

NOZZLE_SNAG_PATH_FRACTIONS = (0.42,)
"""[INFERRED] Normalised path positions of snag events. A single snag matches
the paper's description of one repeatable worst-case point."""

NOZZLE_SNAG_WIDTH = 0.010
"""[INFERRED] Normalised path width over which a snag acts."""

# ---------------------------------------------------------------------------
# Printing process
# ---------------------------------------------------------------------------

NOZZLE_DIAMETER_MM = 1.2
"""[PAPER] Sec. 3: diameters of 0.4, 0.8 and 1.2 mm were all shown to work
free-form; "ultimately, a diameter of 1.2 mm was chosen as it produced
structures with the required level of stiffness"."""

LAYER_HEIGHT_MM = 0.8
"""[PAPER] Sec. 5.3: "A layer height of 0.8 mm, or 75% of the nozzle diameter
was therefore used." """

TCP_SPEED_MM_S = 3.33
"""[PAPER] Sec. 3 and 4.2: printhead movement speed 3.33 mm/s."""

EXTRUSION_SPEED_MM_S = 2.75
"""[PAPER] Sec. 3: extrusion speed 2.75 mm/s, deliberately lower than the
3.33 mm/s printhead speed to keep the molten material permanently under
tension (removing the Wireprint-style pause at each stroke)."""

PRINT_TEMPERATURE_C = 180.0
"""[PAPER] Sec. 3: PLA printed at a lower-than-recommended 180 deg C to reduce
solidification time."""

MAX_FREEFORM_HEIGHT_MM = 5.5
"""[PAPER] Sec. 3: nozzle geometry restricts free-form height to 5.5 mm."""

MAX_FREEFORM_ANGLE_DEG = 45.0
"""[PAPER] Sec. 3: free-form angle must be "less than 45 deg with the
horizontal", else the printhead recontacts printed material (Fig. 3)."""

# ---------------------------------------------------------------------------
# Control / command timing
# ---------------------------------------------------------------------------

ARM_COMMAND_PERIOD_S = 0.060
"""[PAPER] Sec. 4.2: "the period between which commands could be sent to the
robotic arm, in our case ... 60 ms"."""

INTERP_STEP_MM = 0.1998
"""[DERIVED] TCP_SPEED_MM_S * ARM_COMMAND_PERIOD_S = 3.33 * 0.060 = 0.1998 mm.
The paper rounds this to "approximately 0.2 mm between commands" (Sec. 4.2)."""

CONTROL_DT_S = 0.020
"""[INFERRED] Free-flyer pose control loop period, s. Not stated; 50 Hz is
typical for an OptiTrack-in-the-loop ROS control stack and is fast relative to
the 60 ms arm command period."""

SIM_DT_S = 0.002
"""[INFERRED] Integration step, s. Chosen well below THRUSTER_TAU_S and
CONTROL_DT_S for integration accuracy."""

TRACKING_NOISE_M = 1.0e-4
"""[INFERRED] Optical tracking position noise, 1-sigma, m.

The paper says pose is "monitored using a visual tracking system" (OptiTrack)
but gives no accuracy figure. 0.1 mm is representative of a well-calibrated
OptiTrack volume of this size.
"""

TRACKING_NOISE_RAD = 3.0e-4
"""[INFERRED] Optical tracking yaw noise, 1-sigma, rad (~0.017 deg)."""

VELOCITY_FILTER_WINDOW = 5
"""[PAPER, qualitative] Sec. 2: "Finite differencing is used to obtain an
estimate of the free-flyer's velocity which is subsequently smoothed by a
moving average filter." The window length is [INFERRED]; 5 samples at
CONTROL_DT_S = 20 ms gives 100 ms of smoothing.
"""

ARM_REPEATABILITY_M = 5.0e-6
"""[INFERRED] Meca500 arm repeatability, m. Sec. 2 identifies the manipulator
as a Mecademic Meca500; 5 um is its published repeatability. Two orders of
magnitude below the errors of interest, so the arm is effectively ideal."""

ARM_LAG_S = 0.030
"""[INFERRED] First-order lag of the arm following a commanded TCP pose, s.
Half the command period; the paper models the arm as an ideal follower with
"an integrated controller" (Sec. 2)."""


@dataclass(frozen=True)
class PIDGains:
    """PID gains for one axis. Output is force (N) or torque (N m)."""

    kp: float
    ki: float
    kd: float
    integral_limit: float = np.inf


PID_XY = PIDGains(kp=29.1, ki=4.4, kd=38.8, integral_limit=0.6)
"""[INFERRED] Translational PID gains, tuned to the paper's stated performance.

Sec. 2 and 4.1 state that three PID controllers are used for x, y and theta,
but no gains are published. Tuned here from the second-order target
omega_n = 1.2 rad/s, zeta = 0.8 on the 20.2 kg body:
    kp = m omega_n^2      = 20.2 * 1.44  = 29.1 N/m
    kd = 2 zeta m omega_n = 2*0.8*20.2*1.2 = 38.8 N s/m
    ki = kp * omega_n / 8 = 4.4 N/(m s)
The integral term is what rejects the quasi-static residual gravity bias; the
limit prevents windup during a friction stick event. Verified against the
paper's +/- 2 mm station-keeping envelope (Sec. 4.1, Fig. 4).
"""

PID_THETA = PIDGains(kp=1.55, ki=0.24, kd=2.07, integral_limit=0.05)
"""[INFERRED] Attitude PID gains.

From the same omega_n = 1.2 rad/s, zeta = 0.8 target applied to INERTIA_ZZ,
then scaled so the attitude loop matches the accuracy the paper's numbers
imply. That scaling is pinned by the reported per-axis error decomposition,
not chosen freely.

With the arm base offset along body y, a yaw error dtheta appears as nozzle x
error of 400 mm * dtheta, while nozzle y error comes only from body position
error. The paper's dry-run means (Sec. 5.1) are x = 0.68 mm and y = 0.28 mm, so:

    body position error  ~ 0.28 mm per axis
    yaw contribution     ~ 0.68 - 0.28 = 0.40 mm
    => mean yaw error    ~ 0.40 / 400 = 1.0 mrad = 0.057 deg

and the resulting total, sqrt(0.68^2 + 0.28^2) = 0.735 mm, recovers the
paper's quoted 0.77 mm mean. These gains are tuned to reproduce that
decomposition rather than to minimise error, which is the point: the
recreation has to match the paper's *performance*, not beat it.
"""


# ---------------------------------------------------------------------------
# Truss geometry
# ---------------------------------------------------------------------------

SEGMENT_LENGTH_MM = 110.0
"""[PAPER] Sec. 5.1: the dry-run trajectory is that "required to print a
110 mm truss element. This size was chosen as it could comfortably fit within
the workspace of the robotic arm"."""

N_SEGMENTS = 7
"""[PAPER] Sec. 5.2: "A series of 7 segments were printed"."""

TOTAL_TRUSS_LENGTH_MM = 775.0
"""[PAPER] Sec. 5.2 and Conclusion: the 7 segments "together forming a truss
with a total length of 775 mm"."""

SEGMENT_ADVANCE_MM = TOTAL_TRUSS_LENGTH_MM / N_SEGMENTS
"""[DERIVED] 775 / 7 = 110.71 mm of new structure per segment. Note how close
this is to the 110 mm single-element figure, which is the consistency check
that ties Sec. 5.1 and Sec. 5.2 together."""

SEGMENT_OVERLAP_MM = 15.0
"""[INFERRED] Length of the overlap region where a new segment is printed onto
the previous one. Sec. 5.2: "To join the individual segments together, they
are overlapped, as shown in Figure 12." The paper shows this photographically
but gives no dimension."""

TRUSS_DEPTH_MM = 25.0
"""[INFERRED] In-plane depth (y extent) of the truss zigzag. Not dimensioned
in the paper; chosen so the diagonal angle stays inside the 45 deg free-form
limit at the chosen bay pitch."""

TRUSS_BAYS_PER_SEGMENT = 4
"""[INFERRED] Number of zigzag bays per segment, consistent with the node
count visible in Figs. 9-11."""


# ---------------------------------------------------------------------------
# Reported results, used as validation targets
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ReportedResult:
    """A result quoted in the paper, in mm, used to validate the recreation."""

    label: str
    mean_mm: float
    max_mm: float
    source: str


REPORTED: dict[str, ReportedResult] = {
    "dry_uncompensated": ReportedResult(
        "dry run, no compensation", 0.77, 2.35,
        "Sec. 5.1: 'Mean errors of 0.77 and 0.26 mm were measured for the dry "
        "run without and with the compensation algorithm'; 'the maximum error "
        "measured 2.35 mm'.",
    ),
    "dry_compensated": ReportedResult(
        "dry run, compensated", 0.26, 1.53,
        "Sec. 5.1: mean 0.26 mm; 'the maximum error of the dry run with the "
        "compensation algorithm was 1.53 mm'.",
    ),
    "print_uncompensated": ReportedResult(
        "print, no compensation", 8.42, 55.45,
        "Sec. 5.1: 'a mean error of 8.42 mm was measured'; maximum error "
        "'55.45 mm without ... the compensation algorithm'.",
    ),
    "print_compensated": ReportedResult(
        "print, compensated", 0.27, 2.89,
        "Sec. 5.1: 'the mean error with the correction algorithm only saw an "
        "increase of 4%, when compared to the dry run, to 0.27 mm'; maximum "
        "'2.89 mm with the compensation algorithm'.",
    ),
}

DRY_RUN_AXIS_MEANS_MM = {"x": 0.68, "y": 0.28}
"""[PAPER] Sec. 5.1: without compensation "the mean error being over twice as
large in the X direction than in the Y direction, with values of 0.68 and
0.28 mm respectively". The asymmetry arises because the 400 mm TCP offset
converts free-flyer yaw error into x position error."""

STATION_KEEPING_POS_TOL_MM = 2.0
"""[PAPER] Sec. 4.1: "maintaining a position with errors with a magnitude of
approximately +/- 2 mm in the x and y directions"."""

STATION_KEEPING_ATT_TOL_DEG = 0.5
"""[PAPER] Sec. 4.1: "maintains orientation about the z axis within
+/- 0.5 deg when not printing"."""

STATION_KEEPING_DURATION_S = 120.0
"""[PAPER] Fig. 4 caption: free-flyer and nozzle deviation "whilst station
keeping for 120 s"."""

NOZZLE_ERROR_MAX_X_MM = 5.5
"""[PAPER] Sec. 4.1: summing rotational and linear errors, "the error is up to
5.5 mm in the x direction. With a nozzle diameter of 1.2 mm, an error of this
magnitude would prevent structures being printed." """

MEAN_ERROR_REDUCTION_DRY = 0.66
"""[PAPER] Sec. 5.1: "approximately a 66% reduction in mean error"."""

MEAN_ERROR_REDUCTION_PRINT = 0.97
"""[PAPER] Sec. 5.3: during printing the algorithm "reduced the mean and
maximum errors during printing by 97 and 95%, respectively"."""

MAX_ERROR_REDUCTION_PRINT = 0.95
"""[PAPER] Sec. 5.3, as above."""


@dataclass(frozen=True)
class SimConfig:
    """Bundle of everything a run needs, so experiments stay reproducible."""

    seed: int = 0
    dt: float = SIM_DT_S
    control_dt: float = CONTROL_DT_S
    arm_command_period: float = ARM_COMMAND_PERIOD_S
    thruster_tau: float = THRUSTER_TAU_S
    thruster_deadband: float = THRUSTER_DEADBAND_N
    pwm_levels: int = PWM_LEVELS
    tracking_noise_m: float = TRACKING_NOISE_M
    tracking_noise_rad: float = TRACKING_NOISE_RAD
    residual_gravity_max: float = RESIDUAL_GRAVITY_MAX
    nozzle_friction_n: float = NOZZLE_FRICTION_N
    nozzle_snag_n: float = NOZZLE_SNAG_N
    pid_xy: PIDGains = field(default_factory=lambda: PID_XY)
    pid_theta: PIDGains = field(default_factory=lambda: PID_THETA)
