# 4. Math toolkit, part 2: uncertainty

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · [2 Setup](02_setup.md) · [3 Geometry](03_math_geometry.md) ·
**4 Uncertainty** · [5 Field](05_field_model.md) · [6 Camera](06_camera.md) ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · [11 Progress](11_progress.md) · [12 Contributing](12_contributing.md)

Every sensor lies a little. Odometry overestimates a step, the camera is a pixel off, a head servo is half a degree
from where it reports. Good localization doesn't pretend these errors away; it **keeps track of how unsure it is**
and weighs each piece of information by how trustworthy it is.

This chapter covers the probability the code uses, assuming none:

- what "noise" and a "distribution" are;
- the normal (bell-curve) distribution and its σ;
- how to combine independent errors;
- covariance: uncertainty in two directions at once;
- Bayes' rule: updating a belief with new evidence;
- likelihoods, log-likelihoods, and robust kernels;
- odds and log-odds, used for the side belief;
- moving averages, exponential decay and percentiles.

---

## 4.1 Noise and distributions

Measure the same thing many times and you get slightly different numbers. That scatter is **noise**. If you plot
how often each value comes up (a histogram), you get the **distribution** of the measurement.

Two numbers summarize a distribution well enough for most of our work:

- the **mean** (average), written μ ("mu"): where the values center;
- the **standard deviation**, written σ ("sigma"): how spread out they are, roughly "the typical size of an
  error".

The **variance** is σ², the standard deviation squared. Variances are easier to compute with; standard deviations
are easier to picture, because they're in the same units as the measurement (meters, radians).

## 4.2 The normal distribution

Most measurement noise looks like a bell curve: the **normal** (or **Gaussian**) distribution. Its height at value
`x` is

```math
p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\!\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)
```

Reading it piece by piece:

- `(x − μ)` is how far `x` is from the center;
- dividing by σ measures that distance "in sigmas";
- squaring makes it positive, and `exp(−…)` makes the curve fall off quickly in both directions;
- the fraction in front only makes the total area equal 1.

The rule of thumb to remember: about **68%** of values fall within 1σ of the mean, **95%** within 2σ, and **99.7%**
within 3σ. So "σ = 5 cm" means "usually within 5 cm, almost never more than 15 cm off".

In the code, `rng.normal(0.0, sigma, n)` draws `n` random numbers from a normal distribution with mean 0 and
standard deviation `sigma`. The simulator uses it to add realistic noise; the particle filter uses it to spread
its guesses.

## 4.3 Combining independent errors

If a measurement has two independent error sources, say pixel noise with σ₁ and head-angle noise with σ₂, the
total error is **not** σ₁ + σ₂. Independent errors partly cancel, and the rule is that **variances add**:

```math
\sigma_{\text{total}}^2 = \sigma_1^2 + \sigma_2^2
\quad\Longrightarrow\quad
\sigma_{\text{total}} = \sqrt{\sigma_1^2 + \sigma_2^2}
```

This is "adding in quadrature". It's exactly `math.hypot(σ₁, σ₂)`, the same function as the length of a right
triangle's long side. Example: σ₁ = 3 cm and σ₂ = 4 cm combine to 5 cm, not 7 cm. You'll see `hypot` all over the
code for this reason, for example:

- the particle filter's `predict` combines odometry noise with a floor value;
- the side belief widens a ball's uncertainty by how far it could have rolled;
- the simulator combines pixel noise with head-angle noise.

A useful consequence: when one error is much bigger, the small one barely matters. hypot(10, 1) ≈ 10.05.

## 4.4 Covariance: uncertainty in two directions

A point on the floor has an x error and a y error. Often they're not independent. For example, a point the camera
sees far ahead is uncertain mostly **along the line of sight** (how far away is it?) and much less **sideways**
(which direction is it?). The uncertainty is a stretched, tilted ellipse, not a circle.

The **covariance matrix** captures that:

```math
\Sigma = \begin{pmatrix} \sigma_x^2 & c \\ c & \sigma_y^2 \end{pmatrix}
```

- the diagonal entries are the variances in x and y;
- `c`, the **covariance**, says how x and y errors move together: positive if a too-large x tends to come with a
  too-large y, negative if opposite, zero if unrelated.

Picture it as an ellipse: the region where the point probably is.

- The ellipse's **axes** point in the directions of largest and smallest uncertainty.
- Its **half-lengths** are the square roots of the matrix's two **eigenvalues**. An eigenvalue here is just the
  variance along one of those axis directions.

For a 2×2 matrix with entries `a`, `b` (off-diagonal) and `c` (bottom-right), the larger eigenvalue has a closed
form, which the code uses in `_lambda_max` in [`pf.py`](../../../td_localization/td_localization/pf.py):

```math
\lambda_{\max} = \frac{a + c}{2} + \sqrt{\left(\frac{a - c}{2}\right)^2 + b^2}
```

The particle filter only needs this largest value: it treats each point as uncertain by its *worst* direction, to
stay on the safe side.

**Building a tilted covariance.** If you know the uncertainty along the line of sight (σ_r, "range") and across it
(σ_t, "tangential"), and the direction of the line of sight is the angle β, then

```math
\Sigma = R(\beta) \begin{pmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_t^2 \end{pmatrix} R(\beta)^{\mathsf{T}}
```

where `R(β)` is the rotation matrix from [chapter 3](03_math_geometry.md). Multiplied out, that's
`cov[0,0] = cos²β σ_r² + sin²β σ_t²` and so on: exactly the lines in `floor_covariances` in
[`sim.py`](../../../td_field_model/td_field_model/sim.py).

Concrete numbers from that function (camera 1.2 m up, typical head-angle noise): a point 3 m straight ahead has
σ_range ≈ 4.9 cm and σ_lateral ≈ 1.9 cm. At 6 m, σ_range grows to about 17 cm. Far points are much less certain,
and [chapter 6](06_camera.md) explains why.

## 4.5 Bayes' rule: updating a belief

Probability is also a way to describe **belief**: "I'm 70% sure I'm on my own half." Bayes' rule says how a belief
should change when evidence arrives:

```math
P(H \mid D) = \frac{P(D \mid H)\, P(H)}{P(D)}
```

In words, for a hypothesis H (say "I'm at spot A") and data D (what the camera saw):

- **P(H)**, the **prior**: how much I believed H before seeing D.
- **P(D | H)**, the **likelihood**: if H were true, how probable would this observation be? "If I were at A, would
  I expect to see a line right there?"
- **P(H | D)**, the **posterior**: my updated belief after seeing D.
- **P(D)** just makes all the posteriors add up to 1. In practice: compute `likelihood × prior` for every
  hypothesis, then divide each by their total.

**Worked example.** The robot is at one of three spots, A, B or C, each equally likely (prior 1/3 each). The camera
sees a line point. If the robot were at A, the point would land right on a line (likelihood 0.9). At B, a bit off
(0.3). At C, far off (0.05).

| Spot | Prior | Likelihood | Prior × likelihood | Posterior (÷ 0.4167) |
|---|---|---|---|---|
| A | 0.333 | 0.9 | 0.300 | **0.72** |
| B | 0.333 | 0.3 | 0.100 | 0.24 |
| C | 0.333 | 0.05 | 0.017 | 0.04 |
| | | | total 0.4167 | 1.00 |

One observation moved the belief from "no idea" to "probably A". A second observation multiplies in another set of
likelihoods, and so on. **A particle filter is this table, with hundreds of rows, updated many times a second**
([chapter 7](07_particle_filter.md)).

## 4.6 Likelihoods in practice: logs and robust kernels

How do we turn "the point landed 3 cm from a line" into a likelihood number? The natural choice is a normal
distribution of the error `e` with the point's σ:

```math
\text{likelihood} \propto \exp\!\left(-\frac{e^2}{2\sigma^2}\right)
```

("∝" means "proportional to": we drop constant factors, because the normalizing step in 4.5 removes them anyway.)

**Why the code uses logarithms.** With 120 line points per frame, we multiply 120 likelihoods. Many are tiny
numbers, and the product can be smaller than a computer can store (it becomes exactly 0). Taking the logarithm
turns products into sums, and `exp` and squaring into simple arithmetic:

```math
\log(\text{likelihood}) = -\frac{e^2}{2\sigma^2} + \text{constant}
```

So the code adds up **log-likelihoods** and only converts back to weights at the end, after subtracting the largest
value so the biggest weight becomes `exp(0) = 1` (no underflow). You'll see exactly this in the `update` method of [`ParticleFilter`](../../../td_localization/td_localization/pf.py).

**Why not the plain Gaussian: outliers.** Some detected "line" pixels aren't lines at all: a white jersey, a shoe,
a line on the neighboring field. With a Gaussian, one such point 1 m from any line, with σ = 7 cm, contributes
−e²/2σ² ≈ −102, overwhelming all the good points. The filter would throw away the correct guess because of one bad
pixel.

The code uses a **robust kernel**, the Cauchy (or Lorentzian) form, instead:

```math
\log(\text{likelihood}) = -\log\!\left(1 + \frac{e^2}{\sigma^2}\right)
```

Compare, for a point with σ = 0.07 m:

| Distance to nearest line | Gaussian −e²/2σ² | Cauchy −log(1 + e²/σ²) |
|---|---|---|
| 0.02 m | −0.04 | −0.08 |
| 0.05 m | −0.26 | −0.41 |
| 0.10 m | −1.02 | −1.11 |
| 0.20 m | −4.1 | −2.2 |
| 0.50 m | −25.5 | −4.0 |
| 1.00 m | −102.0 | −5.3 |

For small errors the two behave alike, so good points are scored almost the same. For large errors the Cauchy
penalty grows only slowly (logarithmically), so a few bad points can't wreck a good guess. That's what "robust"
means.

## 4.7 Odds and log-odds

For a yes/no question ("am I on the half I think?"), the code tracks the **log-odds** instead of the probability.

- **Odds** = p / (1 − p). A probability of 0.75 is odds of 3 ("3 to 1").
- **Log-odds** = ln(odds) = ln(p / (1 − p)).

To go back, use the **sigmoid** function:

```math
p = \frac{1}{1 + e^{-\ell}} \qquad \text{where } \ell \text{ is the log-odds}
```

| Log-odds ℓ | Probability p |
|---|---|
| −8 | 0.0003 |
| −4 | 0.018 |
| 0 | 0.5 |
| 1 | 0.731 |
| 2.2 | 0.90 |
| 3 | 0.953 |
| 4.6 | 0.99 |
| 6 | 0.9975 |
| 8 | 0.9997 |

Why bother? **In log-odds, Bayes' rule becomes addition.** Each independent piece of evidence contributes a
**log-likelihood ratio** (LLR):

```math
\ell_{\text{new}} = \ell_{\text{old}} + \underbrace{\log\frac{P(D \mid \text{yes})}{P(D \mid \text{no})}}_{\text{LLR}}
```

Evidence for "yes" has a positive LLR, evidence against is negative, and useless evidence is 0. You can add, clip
and decay these numbers easily, which is exactly what the side belief does ([chapter 8](08_side_belief.md)).

Example: start unsure (ℓ = 0, p = 0.5). One teammate's report gives +1.9, a second +1.9: ℓ = 3.8, p ≈ 0.98.

## 4.8 Moving averages and decay

Two small tools from signal processing appear in the code.

**Exponential moving average** (used for the particle filter's "match quality"):

```math
q \leftarrow q + \alpha\,(q_{\text{new}} - q)
```

Each update moves the average a fraction α of the way toward the new value. Small α = slow, smooth average. Large α
= fast, jumpy average. It "remembers" roughly the last 1/α updates. The filter keeps two (α = 0.3 and α = 0.005) and
compares them to notice when things suddenly get worse ([chapter 7](07_particle_filter.md)).

**Exponential decay** (used to fade confidence while the robot is unsure):

```math
\ell \leftarrow \ell \cdot e^{-\Delta t / \tau}
```

After τ seconds, a value has shrunk to 37% (that's e⁻¹); after 3 s with τ = 6 s, to 61%.

## 4.9 Percentiles

To summarize thousands of errors, the evaluation tool reports **percentiles**:

- **p50** (the **median**): half the errors are smaller than this. A typical error.
- **p95**: 95% of errors are smaller. A "bad moment" error.
- **max**: the single worst error.

The targets in the [roadmap](../../roadmap.md) §3.3 are written this way: position p50 ≤ 0.2 m, p95 ≤ 0.5 m, heading
p95 ≤ 5°. Percentiles are used instead of averages because one huge error (say, during a kidnap) would distort an
average, while p50 and p95 stay meaningful.

## 4.10 Try it

```python
import math
import numpy as np

rng = np.random.default_rng(0)
x = rng.normal(0.0, 0.05, 100_000)        # 100,000 errors with σ = 5 cm
print(x.std())                            # ≈ 0.05
print(np.mean(np.abs(x) < 0.05))          # ≈ 0.68 (within 1σ)
print(np.mean(np.abs(x) < 0.10))          # ≈ 0.95 (within 2σ)

# Independent errors add in quadrature
y = rng.normal(0.0, 0.03, 100_000) + rng.normal(0.0, 0.04, 100_000)
print(y.std(), math.hypot(0.03, 0.04))    # both ≈ 0.05

# Gaussian vs Cauchy penalty for a point 1 m off with σ = 0.07 m
e, s = 1.0, 0.07
print(-e**2 / (2 * s**2), -math.log1p(e**2 / s**2))   # ≈ -102.0 vs -5.32

# Log-odds and back
def logodds(p): return math.log(p / (1 - p))
def prob(l): return 1 / (1 + math.exp(-l))
print(logodds(0.9), prob(6.0))            # ≈ 2.197, 0.9975

# Percentiles
errors = np.abs(rng.normal(0.0, 0.1, 1000))
print(np.percentile(errors, 50), np.percentile(errors, 95))   # ≈ 0.071, 0.199
```

## 4.11 Summary

- σ is a typical error size; 68/95/99.7% fall within 1/2/3σ of the mean.
- Independent errors add **in quadrature**: `hypot(σ₁, σ₂)`.
- A **covariance matrix** is an uncertainty ellipse; its largest eigenvalue is the worst-direction variance.
- **Bayes' rule:** posterior ∝ likelihood × prior, then normalize.
- Work in **log-likelihoods** to avoid underflow; use a **robust (Cauchy) kernel** so outliers can't dominate.
- **Log-odds** turn Bayes' rule into adding evidence (LLRs).
- Moving averages smooth, exponential decay fades, percentiles summarize.

---

**Next:** [5. The field model](05_field_model.md)
