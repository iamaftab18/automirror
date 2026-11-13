# Smart Rear-View Mirror – 3D Printed Parts (Bambu Lab A1 mini)

Raspberry Pi 4B + USB camera module + 2 × TowerPro MG995 (Servo 1 = tilt / up-down, Servo 2 = pan / left-right). No motor driver.
All STLs in `stl/` are already in their print orientation. Import them into Bambu Studio and slice. **No supports needed.**

## Parts

| # | File | Size (mm) | What it does |
|---|------|-----------|--------------|
| 1 | `1_enclosure_shell.stl` | 146 × 174 × 52 | Square box with honeycomb mesh walls. Roof flanges with 4 slotted holes (M4/M5). Has the camera window, USB-C opening for the Pi and a DC-jack hole. Prints top-plate-down. |
| 2 | `2_enclosure_tray_pi_base.stl` | 146 × 146 × 10 | Removable bottom plate. Holds the Pi 4B (standoffs), a buck converter (zip-tie slots), vents and the servo cable slot. Screws up into the shell's corner posts. |
| 3 | `3_servo1_tilt_bracket.stl` | 58 × 58 × 64 | Bolts under the centre of the tray. Holds **Servo 1** (shaft horizontal = tilt). Has an M4 pivot on the opposite side so the yoke is supported on **both** sides. |
| 4 | `4_tilt_yoke_servo2_holder.stl` | 68 × 62 × 69 | U-yoke: one arm on Servo 1's horn, one arm on the M4 pivot. Its base holds **Servo 2** (shaft pointing down = pan). |
| 5 | `5_mirror_adapter.stl` | 40 × 36 × 19 | Fixes to Servo 2's horn. Has a V-groove clamp jaw. |
| 6 | `6_mirror_clamp_cap.stl` | 40 × 36 × 14 | The second V-jaw. Together with part 5 it clamps the mirror's stem or arm (round, Ø 6–18 mm) with 4 × M4. |

Tested in CAD: no part touches another through tilt ±35° and pan ±90°. The tilt axis is 48 mm below the box. The bottom of the mirror clamp is about 155 mm below the box.

## Print settings (A1 mini)
- **Material: PETG, not PLA.** A parked car's cabin often gets above 60–70 °C. PLA softens at that heat and the parts will sag.
- 0.4 mm nozzle, 0.2 mm layers, **4 walls**, 5 top and 5 bottom layers, 25–30 % gyroid infill.
- Parts 3, 4 and 5: use 5–6 walls so the screws grip well. A brim on part 3 helps because it is tall.
- Roughly 250–300 g of PETG for the full set. The shell is the biggest print.

## Hardware
| Qty | Item | Used for |
|---|---|---|
| 4 | M3 × 12 pan head | Tray → shell corner posts (they self-tap into the 2.8 mm pilot holes) |
| 4 + 4 | M3 × 12 countersunk + M3 nuts | Tray → Servo 1 bracket (the nuts sit in hex pockets) |
| 8 + 8 | M3 × 12 (M3 × 16 if you use the rubber grommets) + nuts | Servo ears → bracket / yoke |
| 1 | M4 × 16 button head + 2 washers + M4 nyloc nut | Yoke pivot (the head sits in the recess on the inside of the bracket) |
| 4 + 4 | M4 × 20 + M4 nuts | Mirror clamp (the nuts slide into the side slots of part 5) |
| 4 | M2.5 × 6 self-tapping (or M2.5 screws) | Raspberry Pi → standoffs |
| 4 | M2 × 12 + nuts + spacers | Camera board behind the lens window |
| 1 | 5.5 × 2.1 mm panel DC jack (11–12 mm thread) or PG7 gland | 12 V input |
| 1 | Buck converter 12 V → 6 V, ≥ 5 A (XL4015 / 300 W type) | Servo power |

## Assembly
1. **Centre both servos (90°) before you fit the horns.**
2. **Shell:** fit the DC jack in the -X wall. Fit the camera board from the inside: the lens goes through the Ø19 hole and the screws go through the diagonal slots (these fit 25–34 mm square hole patterns, e.g. 32 × 32 and 38 × 38 boards).
3. **Bracket (part 3):** first push the M4 pivot bolt through the bracket's pivot wall from the inside, so the head sits in its recess. Then slide Servo 1 into the window with the shaft facing away from the pivot, and bolt the ears with M3 bolts and nuts.
4. **Yoke (part 4):**
   - Bolt Servo 2 in from below with the shaft pointing down. The nuts go on top of the yoke's base plate.
   - Screw a round horn onto the yoke arm that has the big centre hole, using the small self-tapping horn screws. The engraved rings help you centre 20 mm and 24 mm horns.
   - Press the horn onto Servo 1's spline. Drive the horn's centre screw through the Ø6.5 access hole.
   - Put the other arm on the M4 bolt with a washer on each side, then add the nyloc nut. Tighten it just snug so the yoke still swings freely. Add washers on the inner side if there is end-play.
5. **Mirror adapter (part 5):** screw a horn onto its top face, press it onto Servo 2 and drive the centre screw through the access hole. Slide the M4 nuts into the side slots. Then clamp the mirror stem between parts 5 and 6 with 4 × M4 × 20.
6. **Tray:**
   - Pi on the standoffs. Its USB-C port faces the -Y wall opening.
   - Buck converter in the zip-tie bay.
   - Bracket under the centre with 4 countersunk M3 bolts.
   - Pass both servo cables up through the slot. Leave a loop of slack so the cables don't pull when the mirror tilts. Zip-tie them to the yoke.
7. Plug the camera into a Pi USB port. Close the tray onto the shell with the 4 corner screws. The tray's locating tabs line it up.
8. **Roof:** screw the flanges into the roof metal, rivnuts or existing grab-handle / sun-visor points, using M4/M5 through the slotted holes. Mounting into the headliner board alone will **not** hold a ~600 g assembly in a car that vibrates.

## Wiring (no driver needed)
- Pi: 5 V 3 A via its USB-C, through the opening in the wall (use a car USB-C PD adapter).
- Servos: red and brown wires → buck converter set to **6.0 V**. Never power the servos from the Pi's 5 V pin.
- **Connect the buck converter's GND to a Pi GND pin.** Without this the servo signals will not work.
- Servo 1 signal → GPIO18 (pin 12). Servo 2 signal → GPIO19 (pin 35). Both are hardware-PWM pins. Use `pigpio` so the servos don't jitter.

## Changing dimensions
Everything is generated by `mirror_mount.py` (Python 3 + `pip install manifold3d trimesh numpy`). Edit the parameters at the top and run `python3 mirror_mount.py`; it writes new STLs to `stl/`. The values you are most likely to change:
- `CAM_Y`, `CAM_Z`, `CAM_LENS_D`: camera position and lens hole size.
- `GROOVE_W`: mirror clamp size.
- `SV_*`: servo dimensions, if your MG995 clone measures differently.
- `IN`: box size.
