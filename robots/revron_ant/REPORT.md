# Verifier report: Revron-style antweight vertical spinner

Profile `aws2025`. 0 blockers, 3 warnings, 18 checks passed.

Mass 130.4 g (range 110.1 to 150.7 g, limit 150 g).

## Findings

- **pass** `validity.ok`: Spec is well formed 22 parts, 8 fasteners, 8 wires.
- **warning** `wiring.bend`: Wire bends too tightly Wire `w_sw_wesc` turns 45 degrees with less than 3.2 mm bend radius; it will kink or pull off its solder joint.
  - Fix: Widen the channel at the corner or route it in a straighter line.
- **warning** `mass.may_be_over`: May be over the weight limit 130.4 g (range 110.1 to 150.7 g) against the 150 g limit; 10.6 g of the uncertainty is catalog estimates, weigh the real parts and set `scale_mass_g`.
  - Fix: Weigh real parts and set `scale_mass_g`, or remove mass until the upper bound fits.
- **pass** `interference.overlap`: No parts collide 58 touching or overlapping pairs checked; touching is allowed, overlap deeper than 0.08 mm is not.
- **pass** `interference.sweep`: No joints to sweep No part has a joint.
- **pass** `interference.spin`: Weapon sweep is clear `disc` swept a full turn without touching another part.
- **pass** `retention.declared`: Held by declared method `esc` is held by tape.
- **pass** `retention.declared`: Held by declared method `rx` is held by tape.
- **pass** `retention.declared`: Held by declared method `power_switch` is held by adhesive.
- **pass** `retention.declared`: Held by declared method `wesc` is held by tape.
- **pass** `retention.captive`: Every loose part is held 7 parts checked, 4 held by a declared method, the rest captive in their pockets.
- **pass** `connection.joined`: Every part is attached All 22 parts lead back to the chassis through screws, shared prints, captive pockets, joints or declared connections.
- **pass** `ground.wheels_touch`: Wheels touch the floor All 2 wheels reach the floor plane (z = -14.00 mm).
- **pass** `ground.clearance`: Body clears the floor Lowest body part is 4.0 mm above the floor.
- **pass** `ground.wedge_gap`: Wedge tip height is right `wedge` tip is 0.64 mm above the floor.
- **pass** `ground.stance`: Stable on the floor Centre of mass is 33.0 mm inside the support polygon of wheels and wedge.
- **warning** `ground.wedge_load`: Front rests on the wedge About 39% of the weight sits on the wedge tip because the centre of mass is ahead of the axle; the tip drags and wears and the drive loses traction.
  - Fix: Move the battery and motors back, or add a front skid wheel, or put the axle forward.
- **pass** `legal.footprint`: Fits the square Fits a 127.0 mm square (limit 150 mm).
- **pass** `assembly.order`: Everything can be assembled Every part slides in along a clear straight path. Assembly order: wesc, insert_rr, insert_rl, insert_fr, insert_fl, rx, esc, battery, motor_r, motor_l, chassis_top, frame, lock_pin, weapon_motor, disc, insert_wr, insert_wl, power_switch, wheel_r, wheel_l, wedge, bolt_wm2, bolt_wm1, bolt_rr, bolt_rl, bolt_fr, bolt_fl, bolt_wr, bolt_wl.
- **pass** `claim.holds`: Claim matches the model "weighs 150 g or less" (mass_total_g <= 150)
- **pass** `claim.holds`: Claim matches the model "fits the 150 mm square" (footprint_mm <= 150)

## Mass budget

| Part | Grams | Plus/minus | Source |
|---|---|---|---|
| chassis_bottom | 12.85 | 2.57 | computed |
| chassis_top | 12.40 | 2.48 | computed |
| frame | 7.05 | 1.41 | computed |
| wedge | 10.46 | 2.09 | computed |
| motor_l | 7.50 | 1.12 | catalog |
| motor_r | 7.50 | 1.12 | catalog |
| wheel_l | 3.40 | 0.51 | catalog |
| wheel_r | 3.40 | 0.51 | catalog |
| battery | 12.50 | 1.88 | catalog |
| esc | 3.00 | 0.45 | catalog |
| rx | 4.50 | 0.67 | catalog |
| power_switch | 0.50 | 0.07 | catalog |
| insert_wl | 0.12 | 0.02 | catalog |
| insert_wr | 0.12 | 0.02 | catalog |
| insert_fl | 0.12 | 0.02 | catalog |
| insert_fr | 0.12 | 0.02 | catalog |
| insert_rl | 0.12 | 0.02 | catalog |
| insert_rr | 0.12 | 0.02 | catalog |
| disc | 14.16 | 0.57 | computed |
| weapon_motor | 17.50 | 2.62 | catalog |
| lock_pin | 0.79 | 0.03 | computed |
| wesc | 6.00 | 0.90 | catalog |
| bolt_wl | 0.33 | 0.02 | fastener |
| bolt_wr | 0.33 | 0.02 | fastener |
| bolt_fl | 0.62 | 0.03 | fastener |
| bolt_fr | 0.62 | 0.03 | fastener |
| bolt_rl | 0.57 | 0.03 | fastener |
| bolt_rr | 0.57 | 0.03 | fastener |
| bolt_wm1 | 0.33 | 0.02 | fastener |
| bolt_wm2 | 0.33 | 0.02 | fastener |
| wires | 1.30 | 0.39 | wire |
| allowance | 1.20 | 0.60 | declared |

## Wire routes

| Wire | AWG | Diameter mm | Length mm |
|---|---|---|---|
| w_sw_wesc | 24 | 1.6 | 100 |
| w_wesc_m | 24 | 1.6 | 90 |
| w_bat_sw | 26 | 1.3 | 98 |
| w_sw_esc | 26 | 1.3 | 21 |
| w_esc_ml | 26 | 1.3 | 26 |
| w_esc_mr | 26 | 1.3 | 31 |
| w_rx_esc | 26 | 1.3 | 81 |
| w_rx_wesc | 26 | 1.3 | 45 |
