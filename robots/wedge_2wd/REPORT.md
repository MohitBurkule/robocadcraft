# Verifier report: Reference 2WD wedge

Profile `aws2025`. 0 blockers, 1 warnings, 11 checks passed.

Mass 102.9 g (range 84.4 to 121.4 g, limit 150 g).

## Findings

- **pass** `validity.ok`: Spec is well formed 15 parts, 4 fasteners, 5 wires.
- **pass** `wiring.route`: Every wire has a route 5 wires routed as real-thickness tubes through free space, 258 mm in total, no collisions.
- **pass** `mass.ok`: Within the weight limit 102.9 g (range 84.4 to 121.4 g) against the 150 g limit; 7.0 g of the uncertainty is catalog estimates, weigh the real parts and set `scale_mass_g`.
- **pass** `interference.overlap`: No parts collide 31 touching or overlapping pairs checked; touching is allowed, overlap deeper than 0.08 mm is not.
- **pass** `interference.sweep`: No joints to sweep No part has a joint.
- **pass** `retention.declared`: Held by declared method `esc` is held by tape.
- **pass** `retention.declared`: Held by declared method `rx` is held by tape.
- **pass** `retention.declared`: Held by declared method `power_switch` is held by adhesive.
- **pass** `retention.captive`: Every loose part is held 6 parts checked, 3 held by a declared method, the rest captive in their pockets.
- **pass** `connection.joined`: Touching structural parts are joined 2 touching pairs each have a screw, shared print, joint or declared connection.
- **pass** `claim.holds`: Claim matches the model "weighs 150 g or less" (mass_total_g <= 150)
- **warning** `claim.unknown_metric`: Claim cannot be checked claim `c_fit`: no measured metric `footprint_mm`

## Mass budget

| Part | Grams | Plus/minus | Source |
|---|---|---|---|
| chassis_bottom | 24.24 | 4.85 | computed |
| chassis_top | 23.82 | 4.76 | computed |
| wedge | 7.96 | 1.59 | computed |
| motor_l | 7.50 | 1.12 | catalog |
| motor_r | 7.50 | 1.12 | catalog |
| wheel_l | 3.40 | 0.51 | catalog |
| wheel_r | 3.40 | 0.51 | catalog |
| battery | 12.50 | 1.88 | catalog |
| esc | 3.00 | 0.45 | catalog |
| rx | 4.50 | 0.67 | catalog |
| power_switch | 0.50 | 0.07 | catalog |
| insert_fl | 0.12 | 0.02 | catalog |
| insert_fr | 0.12 | 0.02 | catalog |
| insert_rl | 0.12 | 0.02 | catalog |
| insert_rr | 0.12 | 0.02 | catalog |
| bolt_fl | 0.57 | 0.03 | fastener |
| bolt_fr | 0.57 | 0.03 | fastener |
| bolt_rl | 0.57 | 0.03 | fastener |
| bolt_rr | 0.57 | 0.03 | fastener |
| wires | 0.57 | 0.17 | wire |
| allowance | 1.20 | 0.60 | declared |

## Wire routes

| Wire | AWG | Diameter mm | Length mm |
|---|---|---|---|
| w_bat_sw | 26 | 1.3 | 100 |
| w_sw_esc | 26 | 1.3 | 21 |
| w_esc_ml | 26 | 1.3 | 26 |
| w_esc_mr | 26 | 1.3 | 28 |
| w_rx_esc | 26 | 1.3 | 82 |
