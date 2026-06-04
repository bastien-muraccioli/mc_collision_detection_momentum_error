# CollisionDetectionMomentumError — mc_rtc Global Plugin

A model-based mc_rtc global plugin for joint-torque collision detection.

---

## How It Works

The plugin implements a PI observer on the **generalized momentum** `p = M(q)q̇`.

At each control tick:

1. **Computes the generalized momentum and driving term** — using the robot's inertia matrix `M`, Coriolis matrix `C`, and the gravity/Coriolis vector `g(q, q̇)`:

   ```
   p = M q̇
   γ = τ + Cᵀ q̇ - g
   ```

2. **Runs the PI observer** — propagating the momentum estimate `p̂` and external torque estimate `τ̂_ext`:

   ```
   ṗ̂       = γ + τ̂_ext + α₁ (p - p̂)
   τ̂̇_ext   = α₂ (p - p̂)
   ```

   Compared to `CollisionDetectionPiSliding`, the sliding correction `α₃ sign(p - p̂)` is absent, which reduces chattering at the cost of slower convergence on large momentum errors.

3. **Applies an adaptive threshold** — an `LpfThreshold` low-pass filter tracks `τ̂̇_ext` and a collision is flagged when any joint exceeds `filtered_signal ± offset`.

4. **Sets a datastore flag** — writes `true` to the `"Obstacle detected"` datastore entry so other controllers can react.

---

## Configuration

All observer gains use hardcoded defaults and can be tuned live through the GUI; no YAML configuration is currently required beyond loading the plugin.

Default values:

| Parameter | Default | Description |
|---|---|---|
| `alpha_1` | `40.0` | Momentum error proportional gain |
| `alpha_2` | `400.0` | External torque estimation proportional gain |
| `threshold_offset_` | `3.0` | Fixed band added/subtracted from the filtered signal |
| `threshold_filtering_` | `0.005` | LPF coefficient for the adaptive threshold (0–1) |

---

## Runtime GUI

The plugin exposes a panel under **Plugins → CollisionDetectionMomentumError**:

| Control | Description |
|---|---|
| `alpha_1` | Adjust momentum observer proportional gain live |
| `alpha_2` | Adjust external torque estimation proportional gain live |
| `Threshold offset` | Adjust the detection band half-width live |
| `Threshold filtering` | Adjust the LPF coefficient live |
| `jointShown` | Select which joint index to display in the plots |
| `Collision stop` | Enable/disable writing the detection result to the datastore |
| `Verbose` | Print per-joint collision messages to the console |
| `Add plot` | Open live plots (see below) |

### Live Plots

| Plot | Contents |
|---|---|
| `CollisionDetectionMomentumError_momentum` | `p` and `p̂` for the selected joint |
| `CollisionDetectionMomentumError_tau_ext_hat` | Estimated external torque `τ̂_ext` |
| `CollisionDetectionMomentumError_momentum_dot` | Observer momentum derivative `ṗ̂` |
| `CollisionDetectionMomentumError_momentum_error` | `τ̂̇_ext` with adaptive threshold bounds |

---

## Logged Entries

| Key | Type | Description |
|---|---|---|
| `CollisionDetectionMomentumError_momentum` | `VectorXd` | Generalized momentum `p = Mq̇` |
| `CollisionDetectionMomentumError_momentum_hat` | `VectorXd` | Estimated momentum `p̂` |
| `CollisionDetectionMomentumError_momentum_hat_dot` | `VectorXd` | Momentum estimate derivative `ṗ̂` |
| `CollisionDetectionMomentumError_tau_ext_hat` | `VectorXd` | Estimated external torque `τ̂_ext` |
| `CollisionDetectionMomentumError_tau_ext_hat_dot` | `VectorXd` | Rate of change of estimated external torque `τ̂̇_ext` |
| `CollisionDetectionMomentumError_gamma` | `VectorXd` | Driving term `γ` |
| `CollisionDetectionMomentumError_momentum_error` | `VectorXd` | Momentum error `p - p̂` |
| `CollisionDetectionMomentumError_momentum_error_high` | `VectorXd` | Upper adaptive threshold |
| `CollisionDetectionMomentumError_momentum_error_low` | `VectorXd` | Lower adaptive threshold |
| `CollisionDetectionMomentumError_obstacleDetected` | `bool` | Collision detection flag |

---

## Datastore Interface

| Key | Type | Written by | Description |
|---|---|---|---|
| `"Obstacle detected"` | `bool` | Plugin (`before`) | `true` when any joint's `τ̂̇_ext` exits the adaptive band; only written when *Collision stop* is enabled |

The entry is created automatically on `init` if it does not already exist.

---

## Dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
