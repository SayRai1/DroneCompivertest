# คู่มือระบบ Vision + Control — parrotMinidroneCompetition_HAM

> เอกสารอธิบายโค้ด/สมการ/ตำแหน่งไฟล์ ทั้งหมด สำหรับศึกษาและจูนเอง
> อัปเดต: 2026-06-24

---

## 0. วิธีรัน (Quick start)

| แก้อะไร | วิธีให้มีผล |
|---------|-------------|
| `mpc_racing_controller.m` (ฟังก์ชันภายนอก) | `clear mpc_racing_controller` → Ctrl+D → Run |
| โค้ด detector (ในบล็อก "waypoints 1") | paste ทับในบล็อก → Ctrl+D → Run |
| `variables.m` (ตัวแปร เช่น FOV, crown) | `startVars` → Ctrl+D → Run |
| `startVars.m` (จุดเริ่ม/init) | `startVars` หรือปิด-เปิดโปรเจกต์ |

ลำดับมาตรฐาน: `startVars` → `clear mpc_racing_controller` → **Ctrl+D** (compile) → **Run**

---

## 1. ภาพรวมสถาปัตยกรรม (Data flow)

```
กล้องมองพื้น (downward camera, 120x160)
   │  binarize (G_B_GAIN, BINARIZER_THRESHOLD)
   ▼
[waypoints 1 block]  error_pixel_generator()      ← VISION / DETECTOR
   │  หา "จุด look-ahead" บนเส้น (crown-mask + commitment)
   ▼  error_x_track, error_y_track, orientation_track, flag_track
[MPC block]  mpc_racing_controller()              ← PLANNER / OUTER LOOP
   │  ตัดสินใจ วิ่ง(translate)/หมุน(rotate) + วาง setpoint
   ▼  x_planned, y_planned, z_planned, yaw_planned
[Mux + Bus Assignment]  →  ReferenceValueServerBus { pos_ref, orient_ref, mode }
   │
   ▼
[Controller subsystem]                            ← INNER LOOP (ของ Parrot เดิม)
   ├─ Attitude PD  (คุม roll/pitch จาก pos error)
   └─ Yaw PD       (คุม yaw จาก orient_ref(1))
   ▼  tau_roll, tau_pitch, tau_yaw, thrust
มอเตอร์ 4 ตัว → โดรนเคลื่อน/หมุน
```

**แนวคิดหลัก:** โหมด position (`controlModePosVsOrient = 1`) → planner วางตำแหน่งเป้า (pos_ref) ให้ inner loop ของ Parrot ไล่ตาม + สั่ง yaw แยกผ่าน orient_ref

---

## 2. แผนที่ไฟล์/บล็อก (อยู่ตรงไหน)

| สิ่งที่แก้ | ตำแหน่ง |
|-----------|---------|
| **Planner/Controller** | `controller/mpc_racing_controller.m` (ฟังก์ชันภายนอก เรียกโดย MPC block) |
| **Detector (ใช้งานจริง)** | ฝังในบล็อก **Control System → Path Planning → ... → Image Processing → Waypoints Follower → "waypoints 1"** |
| Detector (paste-source ปัจจุบัน) | `controller/waypoints_1_COMMIT.m` |
| Detector (สำรอง) | `controller/waypoints_1_BRANCHPICK.m`, `waypoints_1_ORIGINAL.m` |
| **พารามิเตอร์ vision** | `variables.m` (FOV, MIN/MAX_RADIUS_CROWN, COG_X/Y, BINARIZER_THRESHOLD) |
| **จุดเริ่ม/init** | `utilities/startVars.m` (`init.posNED`, `init.euler`, Ts, TFinal) |
| **MPC block (Stateflow)** | `Control System/Path Planning/MPC block` — chart เรียก `mpc_racing_controller` |
| **Mux + Bus Assignment (yaw)** | ใน Path Planning (ที่ wire `orient_ref`) |
| **Inner loop Attitude PD** | Controller subsystem → **Attitude** (roll/pitch) |
| **Inner loop Yaw PD** | Controller subsystem → **Yaw** |
| **โมเดล** | `controller/flightControlSystem.slx` |
| **Backup** | `D:\sirapop\studi\drone\backup_parrotHAM_2026-06-23_branchpick\` |

---

## 3. VISION / DETECTOR — `error_pixel_generator`

ไฟล์ paste-source: `controller/waypoints_1_COMMIT.m`

### 3.1 หน้าที่
หา "จุดบนเส้น ที่อยู่ข้างหน้าโดรน" (look-ahead point) จากภาพ binary โดยมองวงแหวน (crown) รอบโดรน แล้วเลือก **แขนเดียว** (กันสับสนตอนมีทางแยก/hairpin)

### 3.2 I/O
```
[error_y_track, error_x_track, orientation_track, flag_track] =
    error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev,
                          MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
```
- `frame` : ภาพ binary (1 = เส้น), ขนาด 120x160
- `x_ref_prev, y_ref_prev` : จุด track เฟรมก่อน (ใช้คำนวณทิศเส้น)
- `flag_track_prev` : เฟรมก่อนเจอเส้นไหม
- **out** `error_x_track` (แกนหน้า), `error_y_track` (แกนข้าง) = ตำแหน่งจุด look-ahead เทียบ COG [px]
- **out** `orientation_track` = ทิศเส้น [rad], `flag_track` = เจอเส้นไหม

### 3.3 สมการ (ตามลำดับการทำงาน)

**(a) Crown mask** — pixel (i,j) อยู่ในวงแหวนไหม:
```
nrm = sqrt( (i-(COG_X-0.5))^2 + (j-(COG_Y-0.5))^2 )
อยู่ในวง ⟺  MIN_RADIUS_CROWN ≤ nrm ≤ MAX_RADIUS_CROWN
```

**(b) มุมของ pixel** (เทียบ COG, image frame):
```
yaw_pt = atan2( i-(COG_X-0.5) , j-(COG_Y-0.5) )
```

**(c) ทิศเส้น (จากจุดเฟรมก่อน)** — เป็น output ด้วย:
```
orientation_track = mod( atan2(-x_ref_prev, y_ref_prev) , 2*pi )
```

**(d) ทิศที่ commit ไว้** (persistent, seed ครั้งแรกจาก orientation_track):
```
ครั้งแรกที่ flag_track_prev=1 :  dir_lock = orientation_track
ref_dir = dir_lock
```

**(e) ตัดแขนข้างหลัง (behind-exclusion)** — wedge รอบ (ref_dir+π):
```
angdiff(a,b) = atan2( sin(a-b), cos(a-b) )           % ผลต่างมุม [-π,π]
เก็บ pixel เมื่อ  | angdiff(yaw_pt, ref_dir+π) | > BEHIND_HALF
```

**(f) เลือกแขน — pixel ที่ใกล้ ref_dir สุด** (Pass 1):
```
branch_yaw = argmin | angdiff(yaw_pt, ref_dir) |   (เฉพาะ pixel ที่ไม่โดนตัดใน (e))
```

**(g) สะสมเฉพาะแขนนั้น** (Pass 2) → centroid:
```
เก็บ pixel เมื่อ  | angdiff(yaw_pt, branch_yaw) | ≤ CLUSTER_HALF
mean_i = Σi / N ,  mean_j = Σj / N
error_x_track = -( round(mean_i) - COG_X )
error_y_track =  ( round(mean_j) - COG_Y )
```

**(h) อัปเดต commitment (low-pass)** — กันแกว่งเฟรมต่อเฟรม:
```
dir_lock = dir_lock + DIR_ALPHA * angdiff(branch_yaw, dir_lock)
```

> **กรณีไม่เจอ:** ถ้า `mean = NaN` (N=0) → error=0, flag_track=0 → controller จะ hold

### 3.4 พารามิเตอร์ + ปุ่มจูน

| ตัวแปร | อยู่ที่ | ค่าปัจจุบัน | ทำอะไร / จูน |
|--------|---------|------------|--------------|
| `MIN/MAX_RADIUS_CROWN` | `variables.m` | 19 / 20 | รัศมีวง = ระยะ look-ahead. เล็ก=มองใกล้/เลี้ยวคม แต่หลุดง่าย; ใหญ่=มองไกล/นิ่ง |
| `FOV` | `variables.m` | 2.0 | (เวอร์ชัน COMMIT ไม่ใช้ arc แล้ว ใช้ behind-exclusion แทน) |
| `COG_X, COG_Y` | `variables.m` | 60, 80 | กลางภาพ = ตำแหน่งโดรน (อย่าแก้) |
| `CLUSTER_HALF` | ในโค้ด detector | `pi/4` (45°) | ความกว้างของ "แขนเดียว" |
| `BEHIND_HALF` | ในโค้ด detector | `pi/3` (60°) | ตัดแขนหลังกว้างแค่ไหน (ใหญ่=ตัดเยอะ) |
| `DIR_ALPHA` | ในโค้ด detector | `0.25` | ความหนึบของทิศ commit. **เล็ก=ล็อกแน่น/ไม่แกว่ง**, ใหญ่=ไวแต่ลังเล |

**แก้ "ลังเลที่ทางแยก":** `DIR_ALPHA` ↓ (0.25→0.15) และ/หรือ `BEHIND_HALF` ↑

---

## 4. PLANNER / CONTROLLER — `mpc_racing_controller.m`

### 4.1 หน้าที่ + I/O
รับจุด look-ahead (error_x/y) → ตัดสินใจ **วิ่ง(TRANSLATE) หรือ หมุน(ROTATE)** → ส่ง pos_ref + yaw_ref
```
[x_planned, y_planned, z_planned, yaw_planned] =
    mpc_racing_controller(x_err, y_err, phi, theta, psi,
                          vx, vy, vz, p, q, r, takeoff_flag, x_est, y_est)
```
ใช้จริง: `x_err, y_err, psi, vx, vy, x_est, y_est, takeoff_flag` (ที่เหลือ interface เฉยๆ)

### 4.2 Heading error + filter
```
bearing_body   = atan2(y_err, x_err)                                  % มุมไปจุด look-ahead (body)
bearing_smooth = (1-YAW_ALPHA)*bearing_smooth + YAW_ALPHA*bearing_body % EMA low-pass
e_abs          = |bearing_smooth|                                     % ขนาด heading error
```
`bearing_smooth` = "ทิศที่ต้องเลี้ยว" หลังกรอง noise (เทอมหลักของทั้ง yaw และ supervisor)

### 4.3 Supervisor — เลือกโหมด (prefer ตรง + ยืนยันก่อนหมุน)
```
ถ้า e_abs > TH_HI :  rot_count = rot_count + 1     % นับเฟรมที่โค้งแรง
ไม่งั้น           :  rot_count = 0
ถ้า rot_count ≥ ROT_DWELL :  mode = ROTATE          % โค้งแรง "ต่อเนื่อง" = โค้งจริง
ไม่งั้นถ้า e_abs < TH_LO   :  mode = TRANSLATE       % ตรงแล้ว = วิ่ง (ดีฟอลต์)
ไม่งั้น                   :  คงโหมดเดิม (hysteresis)
```
- **TH_HI ≠ TH_LO** = hysteresis กันสลับถี่
- **ROT_DWELL** = ต้องโค้งแรงค้าง N เฟรมก่อนยอมหมุน → กัน noise สั่ง หมุนมั่ว ("รอจังหวะที่ใช่")

### 4.4 Speed profile
```
ถ้า ROTATE :  v_along = 0                                  % หยุด หมุนหัวก่อน
ไม่งั้น     :  v_along = V_MAX * max(0, 1 - e_abs/TH_FULL)   % โค้งมาก = ช้าลง
```

### 4.5 Yaw command
```
yaw_raw     = psi + YAW_GAIN * bearing_smooth
yaw_planned = atan2( sin(yaw_raw), cos(yaw_raw) )          % wrap [-π,π] กัน overshoot 2π
```

### 4.6 Position setpoint + velocity damping (ลด overshoot)
```
vN = vx*cos(psi) - vy*sin(psi)        % ความเร็ว body → NED
vE = vx*sin(psi) + vy*cos(psi)
bearing_ned = psi + bearing_body                          % ทิศไปจุด look-ahead ใน NED
x_planned = x_est + v_along*cos(bearing_ned) - K_DAMP*vN   % เล็งหน้า - เบรกตามความเร็ว
y_planned = y_est + v_along*sin(bearing_ned) - K_DAMP*vE
z_planned = Z_TRACK
```
เทอม `-K_DAMP*v` = เล็ง setpoint "ถอยหลังจาก momentum" → เบรก → overshoot ลด

### 4.7 Track lost / takeoff (else)
```
x_planned=x_est; y_planned=y_est; yaw_planned=psi          % ค้างอยู่กับที่
bearing_smooth=0; mode=TRANSLATE; rot_count=0              % reset state กัน spike ตอนเจอเส้นใหม่
```

### 4.8 พารามิเตอร์ + ปุ่มจูน (`mpc_racing_controller.m:43-52`)

| ตัวแปร | ค่า | ทำอะไร | จูนเพิ่ม |
|--------|-----|--------|----------|
| `V_MAX` | 0.19 | ความเร็วทางตรง | เร็วขึ้น → 0.25-0.35 |
| `YAW_GAIN` | 0.6 | แรงหันหัวเข้าเส้น | คมขึ้น → 0.8 / นุ่ม → 0.4 |
| `YAW_ALPHA` | 0.30 | กรอง noise ของ yaw | นุ่มขึ้น → 0.15 (เล็ก=นิ่ง/ช้า) |
| `TH_HI` | 0.87 (~50°) | ขีดเริ่มหมุน | หมุนยากขึ้น(ตรงเยอะ) → 1.0 |
| `TH_LO` | 0.35 (~20°) | ขีดกลับมาวิ่ง | — |
| `TH_FULL` | 1.40 (~80°) | โค้งแล้วช้าแค่ไหน | เล็ก=ช้าตอนโค้งเยอะ |
| `ROT_DWELL` | 8 | กี่เฟรมยืนยันก่อนหมุน | หมุนน้อยลง → 12-15 |
| `K_DAMP` | 0.30 | เบรก overshoot | overshoot ↓ → 0.40 / ตรงเร็วขึ้น → 0.20 |
| `ERR_MIN` | 2.5 | px ขั้นต่ำถือว่าเจอเส้น | — |
| `Z_TRACK` | -1.1 | ความสูงบิน [m NED] | — |

---

## 5. การต่อสายในโมเดล (Simulink wiring ที่แก้ไป)

### 5.1 MPC block — 4 output (ชื่อ "ปลอม" เพื่อรักษา Mux เดิม)
chart เรียก:
```
[roll_ref, pitch_ref, yaw_ref, thrust] = mpc_racing_controller(...)
```
**แต่ค่าจริงคือ** (position mode):
```
roll_ref  = x_planned   (out1)
pitch_ref = y_planned   (out2)
yaw_ref   = z_planned   (out3)
thrust    = yaw_planned (out4)   ← yaw ซ่อนในตัวชื่อ "thrust"!
```

### 5.2 การต่อ (ใน Path Planning)
```
out1,2,3 ──► Mux ──► Bus Assignment [assign pos_ref]      (ของเดิม)
out4 (thrust=yaw) ─┐
   Constant 0 ─────┼─► Mux(3) ─► Data Type Conv ─► Bus Assignment [assign orient_ref]
   Constant 0 ─────┘
```
**สำคัญ:** ลำดับ Mux ต้องให้ `thrust` อยู่ **input 1** (= orient_ref element 1)

### 5.3 Convention ของ Parrot (จุดที่เคยพลาด)
`orient_ref` ของ Parrot **= [yaw, pitch, roll]** (yaw มาก่อน!) — ไม่ใช่ [roll,pitch,yaw]
ยืนยันจาก Selector ในโมเดล (ดึง index 1 ส่งเข้า Yaw subsystem)
→ yaw_planned ต้องไป `orient_ref(1)` = Mux **input 1**

---

## 6. Inner-loop controllers (ของ Parrot เดิม — ไม่ได้แก้ แต่ควรรู้ค่า)

### 6.1 Attitude PD (roll/pitch) — `Controller/Attitude`
```
error_pitchroll = refAttitude - [pitch; roll]
P_pr   = [0.013; 0.011]        % สัดส่วน (pitch; roll)
D_pr   = [0.002; 0.003]        % อนุพันธ์ บน [q; p]
I_pr   = 0.01                  % อินทิกรัล (limit ±2, anti-windup 0.001)
→ tau_pitch, tau_roll
```

### 6.2 Yaw PD — `Controller/Yaw`
```
error_yaw = yaw_ref - yaw          % yaw_ref = orient_ref(1) = yaw_planned ของเรา
P_yaw = 0.004
D_yaw = 0.3*0.004 = 0.0012         % บน yaw-rate r
→ tau_yaw
```
> ถ้าโดรน "หมุนช้า/ไม่ทันโค้ง" แม้ yaw_planned ถูก → เพิ่ม `P_yaw` ใน subsystem นี้ (เช่น 0.004→0.006)

---

## 7. จุดเริ่ม + Sim settings (`utilities/startVars.m`)

```
init.posNED = [0.5  1  -0.046]     % จุด spawn (North, East, Down) [m]
init.euler  = [0 0 0]              % มุมเริ่ม (roll,pitch,yaw) — yaw=0 = หันทิศ North(+X)
Ts          = 0.005                % step ของ controller [s]
VTs         = 40*Ts = 0.2          % step ของ vision [s]  (กล้องอัปเดตช้ากว่า 40 เท่า!)
TFinal      = 100                  % เวลา sim [s]
takeOffDuration = 1
```
> **ถ้าเปลี่ยนสนามแล้วโดรนไม่เจอเส้น** → เส้นเริ่มไม่ตรง (0.5,1) → แก้ `init.posNED` ให้ตรงจุดเริ่มเส้น (หาพิกัดจาก `openTrackBuilder`)
> **VTs=0.2s สำคัญ:** detector อัปเดตทุก 0.2s เท่านั้น → ค่า EMA/dwell ในหน่วย "เฟรม vision" ไม่ใช่ step controller

---

## 8. ตารางจูนรวม (เรียงตามอาการ)

| อาการ | แก้ตัวไหน | ทิศ | ไฟล์ |
|-------|-----------|-----|------|
| ช้าทางตรง | `V_MAX` ↑ / `K_DAMP` ↓ | 0.30 / 0.20 | controller |
| overshoot โค้ง | `K_DAMP` ↑ / `TH_FULL` ↓ | 0.40 | controller |
| หมุนบ่อย/มั่ว | `ROT_DWELL` ↑ / `TH_HI` ↑ | 12 / 1.0 | controller |
| หมุนช้าไม่ทันโค้ง | `YAW_GAIN` ↑ / `P_yaw` ↑ | 0.8 / 0.006 | controller / Yaw block |
| yaw สั่น | `YAW_ALPHA` ↓ | 0.15 | controller |
| ลังเลที่ทางแยก | `DIR_ALPHA` ↓ / `BEHIND_HALF` ↑ | 0.15 | detector |
| หลุดเส้นง่าย | `MAX_RADIUS_CROWN` ↑ | 24 | variables.m |
| เลี้ยวคม | `MIN/MAX_RADIUS_CROWN` ↓ | 14/15 | variables.m |

---

## 9. สมการรวม (Reference)

```
angdiff(a,b) = atan2(sin(a-b), cos(a-b))                 % ผลต่างมุม [-π,π]

── DETECTOR ──
nrm        = sqrt((i-(COG_X-.5))^2 + (j-(COG_Y-.5))^2)
yaw_pt     = atan2(i-(COG_X-.5), j-(COG_Y-.5))
keep       = |angdiff(yaw_pt, dir_lock+π)| > BEHIND_HALF        (behind-exclusion)
branch_yaw = argmin|angdiff(yaw_pt, dir_lock)|  over keep       (เลือกแขน)
cluster    = |angdiff(yaw_pt, branch_yaw)| ≤ CLUSTER_HALF
err_x      = -(round(mean_i)-COG_X) ;  err_y = round(mean_j)-COG_Y
dir_lock  += DIR_ALPHA * angdiff(branch_yaw, dir_lock)          (commit)

── CONTROLLER ──
bearing_body   = atan2(y_err, x_err)
bearing_smooth = (1-α)·bearing_smooth + α·bearing_body ,  α=YAW_ALPHA
e_abs          = |bearing_smooth|
rot_count      = (e_abs>TH_HI) ? rot_count+1 : 0
mode           = (rot_count≥ROT_DWELL)?ROTATE : (e_abs<TH_LO)?TRANSLATE : mode
v_along        = ROTATE ? 0 : V_MAX·max(0, 1 - e_abs/TH_FULL)
yaw_planned    = wrap(psi + YAW_GAIN·bearing_smooth)
vN,vE          = R(psi)·[vx;vy]
x_planned      = x_est + v_along·cos(psi+bearing_body) - K_DAMP·vN
y_planned      = y_est + v_along·sin(psi+bearing_body) - K_DAMP·vE
z_planned      = Z_TRACK

── INNER (Parrot เดิม) ──
tau_pr  = P_pr·(ref-[pitch;roll]) + I_pr·∫ - D_pr·[q;p]
tau_yaw = P_yaw·(yaw_ref - yaw)   - D_yaw·r
```

---

## 10. Backup / Revert

โฟลเดอร์: `D:\sirapop\studi\drone\backup_parrotHAM_2026-06-23_branchpick\`

| ต้องการ | ทำ |
|---------|-----|
| คืน controller ก่อน supervisor | copy `mpc_racing_controller_preSupervisor.m` → ทับเป็น `controller/mpc_racing_controller.m` (ต้องชื่อนี้!) |
| คืน detector | paste `waypoints_1_BRANCHPICK.m` หรือ `waypoints_1_ORIGINAL.m` กลับเข้าบล็อก |
| คืนทั้งโมเดล | ปิด Simulink → copy `flightControlSystem.slx` ทับ |

> **กฎ MATLAB:** ฟังก์ชันเรียกตาม **ชื่อไฟล์** — ตัวที่ใช้งานต้องชื่อ `mpc_racing_controller.m` เสมอ (ชื่อ `_preSupervisor` เป็นแค่ backup เรียกตรงไม่ได้)

---

*จบคู่มือ — แก้/จูนได้ตามตารางข้อ 8 ทุกตัวมีตำแหน่งไฟล์กำกับ*
