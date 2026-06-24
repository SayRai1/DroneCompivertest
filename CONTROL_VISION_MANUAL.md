# คู่มือระบบ Vision + Control — parrotMinidroneCompetition_HAM

> เอกสารอธิบายโค้ด/สมการ/ตำแหน่งไฟล์ ทั้งหมด สำหรับศึกษาและจูนเอง
> อัปเดต: 2026-06-24 (controller = stable crab + guide, **ไม่มี yaw แล้ว**)

---

## 0. วิธีรัน (Quick start)

| แก้อะไร | วิธีให้มีผล |
|---------|-------------|
| `mpc_racing_controller.m` (ฟังก์ชันภายนอก) | `clear mpc_racing_controller` → Ctrl+D → Run |
| โค้ด detector (ในบล็อก "waypoints 1") | paste ทับในบล็อก → Ctrl+D → Run |
| `variables.m` (FOV, crown ฯลฯ) | `startVars` → Ctrl+D → Run |
| `startVars.m` (จุดเริ่ม/init) | `startVars` หรือปิด-เปิดโปรเจกต์ |

⚠️ **สำคัญ:** sim จริงรันได้เฉพาะใน MATLAB ที่ลง **Parrot Minidrone Support Package** (มี `parrotlib`) — เครื่องที่ไม่มีจะรันฟังก์ชันโดดๆ ได้แต่รันโมเดลไม่ได้

---

## 1. ภาพรวมสถาปัตยกรรม (Data flow)

```
กล้องมองพื้น (downward camera, 120x160)
   │  binarize (G_B_GAIN, BINARIZER_THRESHOLD) + Erode
   ▼
[waypoints 1 block]  error_pixel_generator()           ← VISION / DETECTOR
   │  หา "จุด look-ahead" บนเส้น (crown-mask + commitment + behind-exclusion)
   ▼  error_x_track, error_y_track, (orientation_track→ทิ้ง), flag_track
[MPC block]  mpc_racing_controller()                   ← PLANNER (OUTER LOOP)
   │  วาง setpoint แบบ "บินเฉียงเข้าหาจุด (holonomic crab)" + ค้าง heading
   ▼  x_planned, y_planned, z_planned, yaw_planned(=psi คงที่)
[Mux + Bus Assignment]  →  ReferenceValueServerBus { pos_ref, orient_ref, mode }
   │
   ▼
[Controller subsystem]                                 ← INNER LOOP (Parrot เดิม)
   ├─ Attitude PD  (คุม roll/pitch จาก pos error)
   └─ Yaw PD       (คุม yaw จาก orient_ref(1) = yaw_planned = psi = "ค้างที่เดิม")
   ▼  tau_roll, tau_pitch, tau_yaw, thrust → มอเตอร์
```

**แนวคิดปัจจุบัน:** position mode — โดรน **บินเฉียง (translate ไปทุกทิศโดยไม่หมุนหัว)** เข้าหาจุดบนเส้น ที่ระยะ look-ahead สั้นๆ → นิ่ง ไม่ปั่น
> **ทำไมไม่มี yaw:** การเปิด yaw ทำให้เกิด spin/tumble ทุกครั้ง (ดู §11) → ตัดออก ใช้ crab อย่างเดียว

---

## 2. แผนที่ไฟล์/บล็อก (อยู่ตรงไหน)

| สิ่งที่แก้ | ตำแหน่ง |
|-----------|---------|
| **Planner/Controller (ใช้งาน)** | `controller/mpc_racing_controller.m` |
| **Detector (ใช้งานจริง)** | ฝังในบล็อก **Image Processing → Waypoints Follower → "waypoints 1"** |
| Detector paste-source (ปัจจุบัน) | `controller/waypoints_1_COMMIT.m` |
| Detector สำรอง | `controller/waypoints_1_BRANCHPICK.m`, `waypoints_1_ORIGINAL.m` |
| **พารามิเตอร์ vision** | `variables.m` (FOV, MIN/MAX_RADIUS_CROWN, COG_X/Y, BINARIZER_THRESHOLD) |
| **จุดเริ่ม/init** | `utilities/startVars.m` (`init.posNED`, `Ts`, `VTs`, `TFinal`) |
| **MPC block (Stateflow)** | `Control System/Path Planning/MPC block` |
| **Mux + Bus Assignment (yaw)** | ใน Path Planning (wire `orient_ref` — ตอนนี้ dormant) |
| **Inner Attitude PD** | Controller subsystem → **Attitude** |
| **Inner Yaw PD** | Controller subsystem → **Yaw** |
| **โมเดล** | `controller/flightControlSystem.slx` |
| **Backup** | `D:\sirapop\studi\drone\backup_parrotHAM_2026-06-23_branchpick\` (ดู §10) |

---

## 3. VISION / DETECTOR — `error_pixel_generator` (COMMIT)

paste-source: `controller/waypoints_1_COMMIT.m`

### 3.1 หน้าที่
หา "จุดบนเส้นข้างหน้าโดรน" (look-ahead) จากภาพ binary โดยมองวงแหวน (crown) รอบโดรน แล้ว **เลือกแขนเดียว** (กันสับสนตอนมีทางแยก/หักศอก)

### 3.2 I/O
```
[error_y_track, error_x_track, orientation_track, flag_track] =
    error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev,
                          MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
```
- `error_x_track`(แกนหน้า), `error_y_track`(แกนข้าง) = ตำแหน่งจุด look-ahead เทียบ COG [px]
- `orientation_track` = ทิศเส้น (ใช้ภายใน seed dir_lock; **output ถูก terminate ทิ้ง** — ไม่ใช้ข้างนอก)
- `flag_track` = เจอเส้นไหม

### 3.3 สมการ (ตามลำดับ)
```
angdiff(a,b) = atan2(sin(a-b), cos(a-b))                       % ผลต่างมุม [-π,π]

(a) crown:   nrm = sqrt((i-(COG_X-.5))^2 + (j-(COG_Y-.5))^2)
             ในวง ⟺ MIN_RADIUS_CROWN ≤ nrm ≤ MAX_RADIUS_CROWN
(b) มุม px:  yaw_pt = atan2(i-(COG_X-.5), j-(COG_Y-.5))
(c) ทิศเส้น: orientation_track = mod(atan2(-x_ref_prev, y_ref_prev), 2π)
(d) commit:  ครั้งแรก dir_lock = orientation_track ;  ref_dir = dir_lock
(e) ตัดหลัง: เก็บ px เมื่อ |angdiff(yaw_pt, ref_dir+π)| > BEHIND_HALF
(f) เลือก:   branch_yaw = argmin|angdiff(yaw_pt, ref_dir)|  (เฉพาะ px ที่ผ่าน e)
(g) สะสม:    เก็บ px เมื่อ |angdiff(yaw_pt, branch_yaw)| ≤ CLUSTER_HALF
             error_x = -(round(mean_i)-COG_X) ;  error_y = round(mean_j)-COG_Y
(h) อัปเดต:  dir_lock += DIR_ALPHA * angdiff(branch_yaw, dir_lock)
```

### 3.4 พารามิเตอร์ + ปุ่มจูน
| ตัวแปร | อยู่ที่ | ค่า | จูน |
|--------|---------|-----|-----|
| `MIN/MAX_RADIUS_CROWN` | `variables.m` | 19 / 20 | รัศมีวง = ระยะ look-ahead (เล็ก=ใกล้/เลี้ยวคม แต่หลุดง่าย) |
| `FOV` | `variables.m` | 2.0 | (COMMIT ใช้ behind-exclusion แทน arc แล้ว) |
| `COG_X, COG_Y` | `variables.m` | 60, 80 | กลางภาพ (อย่าแก้) |
| `CLUSTER_HALF` | โค้ด detector | `pi/4` | ความกว้าง "แขนเดียว" |
| `BEHIND_HALF` | โค้ด detector | `pi/3` | ตัดแขนหลังกว้างแค่ไหน (ใหญ่=ตัดเยอะ) |
| `DIR_ALPHA` | โค้ด detector | `0.25` | ความหนึบของ commit (เล็ก=ล็อกแน่น/ไม่แกว่ง) |

---

## 4. PLANNER / CONTROLLER — `mpc_racing_controller.m` (ปัจจุบัน)

### 4.1 หน้าที่ — Holonomic crab + guide + corner slow-down (**ไม่มี yaw, ไม่มี damping**)
รับจุด look-ahead → วาง setpoint ระยะสั้นๆ ไปทางจุดนั้นใน NED → โดรนบินเฉียงเข้าหา (ไม่หมุนหัว) → ตามเส้นด้วยการ translate
```
[x_planned, y_planned, z_planned, yaw_planned] =
    mpc_racing_controller(x_err, y_err, phi, theta, psi, vx..r, takeoff_flag, x_est, y_est)
```
ใช้จริง: `x_err, y_err, psi, x_est, y_est, takeoff_flag`

### 4.2 สมการ
```
bearing_body = atan2(y_err, x_err)                            % ทิศไปจุด look-ahead (body)

% guide = ทิศ smooth (wrap-safe EMA) ดูดซับการ flip ของ detector
d              = angdiff(bearing_body, bearing_smooth)
bearing_smooth = bearing_smooth + B_ALPHA * d

% corner slow-down: มุมเยอะ → look-ahead สั้น → crab ค่อยๆ ลงเกาะแขนใหม่ (ไม่ orbit)
v = (|bearing_smooth| > TH_SLOW) ? L_SLOW : LOOKAHEAD

% setpoint = จุดข้างหน้า v เมตร ตามทิศ guide ใน NED
bearing_ned = psi + bearing_smooth
x_planned   = x_est + v * cos(bearing_ned)
y_planned   = y_est + v * sin(bearing_ned)
yaw_planned = psi                                            % ★ ค้าง heading = ไม่มี yaw
z_planned   = Z_TRACK
```
track lost/takeoff → hold pose + `bearing_smooth = 0`

### 4.3 พารามิเตอร์ + ปุ่มจูน (`mpc_racing_controller.m`)
| ตัวแปร | ค่า | ทำอะไร | จูน |
|--------|-----|--------|-----|
| `LOOKAHEAD` | 0.12 | ระยะ/ความเร็วทางตรง [m] | เร็วขึ้น → 0.16-0.20 |
| `L_SLOW` | 0.05 | ระยะ/ความเร็วตอนมุม [m] | เลี้ยวไวขึ้น → 0.07 |
| `TH_SLOW` | 0.60 (~34°) | มุมเกินนี้ = เริ่มชะลอ | ชะลอไวขึ้น → 0.45 |
| `B_ALPHA` | 0.25 | guide smooth (เล็ก=นิ่ง/ดูดซับ flip มากขึ้น) | สั่น → 0.15 |
| `ERR_MIN` | 2.5 | px ขั้นต่ำถือว่าเจอเส้น | — |
| `Z_TRACK` | -1.1 | ความสูงบิน [m NED] | — |

---

## 5. การต่อสายในโมเดล (Simulink) — yaw path (ตอนนี้ **dormant**)

> ยัง wire ไว้ แต่ controller ส่ง `yaw_planned = psi` → Yaw PD แค่ "ค้าง heading เดิม" (ไม่หมุน) เก็บไว้เผื่ออยากเปิด yaw อีกในอนาคต

### 5.1 MPC block — 4 output ชื่อ "ปลอม" (รักษา Mux เดิม)
```
roll_ref = x_planned · pitch_ref = y_planned · yaw_ref = z_planned · thrust = yaw_planned
                                                                      ↑ yaw ซ่อนในชื่อ "thrust"!
```
### 5.2 การต่อ
```
out1,2,3 ─► Mux ─► Bus Assignment [pos_ref]
out4(thrust=yaw) + Const0 + Const0 ─► Mux ─► Data Type Conv ─► Bus Assignment [orient_ref]
```
### 5.3 Convention Parrot (จุดที่เคยพลาด)
`orient_ref = [yaw, pitch, roll]` (**yaw มาก่อน!**) → yaw_planned ต้องไป **Mux input 1** = orient_ref(1)

---

## 6. Inner-loop controllers (Parrot เดิม — ไม่ได้แก้)
**Attitude PD** (`Controller/Attitude`): `P_pr=[0.013;0.011]`, `D_pr=[0.002;0.003]`, `I_pr=0.01` → tau_pitch, tau_roll
**Yaw PD** (`Controller/Yaw`): `P_yaw=0.004`, `D_yaw=0.3*0.004` → tau_yaw (ตอนนี้ ref=psi คงที่ = แค่ hold)

---

## 7. จุดเริ่ม + Sim (`utilities/startVars.m`)
```
init.posNED = [0.5  1  -0.046]     % จุด spawn (N,E,D) [m] — ถ้าเปลี่ยนสนามต้องตั้งให้ตรงจุดเริ่มเส้น
init.euler  = [0 0 0]              % yaw=0 = หันทิศ North(+X)
Ts  = 0.005   ;  VTs = 40*Ts = 0.2 ;  TFinal = 100
```
> **VTs=0.2s:** กล้อง/detector อัปเดตทุก 0.2s (ช้ากว่า controller 40 เท่า) — สาเหตุหลักที่ yaw control ไม่เสถียร (§11)

---

## 8. ตารางจูนรวม (ตามอาการ)
| อาการ | knob | ทิศ | ไฟล์ |
|-------|------|-----|------|
| ช้าทางตรง | `LOOKAHEAD`↑ | 0.16-0.20 | controller |
| เลี้ยวมุมไม่ทัน/ช้า | `L_SLOW`↑ หรือ `TH_SLOW`↓ | 0.07 / 0.45 | controller |
| ส่าย/แกว่งตอนมุม | `B_ALPHA`↓ | 0.15 | controller |
| ลังเลที่ทางแยก/หักศอก | `DIR_ALPHA`↓ / `BEHIND_HALF`↑ | 0.15 | detector (COMMIT) |
| หลุดเส้นง่าย | `MAX_RADIUS_CROWN`↑ | 24 | variables.m |
| เลี้ยวคม/มองใกล้ | `MIN/MAX_RADIUS_CROWN`↓ | 14/15 | variables.m |

---

## 9. สมการรวม (Reference)
```
angdiff(a,b) = atan2(sin(a-b), cos(a-b))

── DETECTOR (COMMIT) ──
keep       = |angdiff(yaw_pt, dir_lock+π)| > BEHIND_HALF      (ตัดแขนหลัง)
branch_yaw = argmin|angdiff(yaw_pt, dir_lock)| over keep
err_x      = -(round(mean_i)-COG_X) ;  err_y = round(mean_j)-COG_Y
dir_lock  += DIR_ALPHA·angdiff(branch_yaw, dir_lock)

── CONTROLLER (crab + guide) ──
bearing_body   = atan2(y_err, x_err)
bearing_smooth += B_ALPHA·angdiff(bearing_body, bearing_smooth)
v              = (|bearing_smooth|>TH_SLOW) ? L_SLOW : LOOKAHEAD
x_planned      = x_est + v·cos(psi+bearing_smooth)
y_planned      = y_est + v·sin(psi+bearing_smooth)
yaw_planned    = psi                                          (no yaw)

── INNER (Parrot) ──
tau_pr  = P_pr·(ref-[pitch;roll]) + I_pr·∫ - D_pr·[q;p]
tau_yaw = P_yaw·(yaw_ref - yaw)   - D_yaw·r
```

---

## 10. Backup / Revert
โฟลเดอร์: `D:\sirapop\studi\drone\backup_parrotHAM_2026-06-23_branchpick\`

| ไฟล์ backup | คืออะไร |
|-------------|---------|
| `mpc_racing_controller_STABLEbaseline.m` | **crab ล้วน no-yaw** (นิ่งแน่ — ตัวกู้ภัย) |
| `mpc_racing_controller_preReset.m` | เวอร์ชัน supervisor+yaw+damping เต็ม (พัง tumble) |
| `mpc_racing_controller_preSupervisor.m` | เวอร์ชัน yaw หมุนแต่ไม่นิ่ง |
| `flightControlSystem.slx` / `variables.m` | สแนปช็อตยุค branch-pick |

**กฎ revert:** copy ไฟล์ทับเป็น `controller/mpc_racing_controller.m` (ต้องชื่อนี้!) → `clear mpc_racing_controller` → Ctrl+D
detector: paste `waypoints_1_*.m` กลับเข้าบล็อก

---

## 11. บทเรียน / ประวัติการแก้ (สำคัญ — อ่านก่อนจะเพิ่มอะไร)

**สิ่งที่ลองแล้ว "พัง" และถูกตัดออก:**
| เพิ่มอะไร | ผล | สาเหตุ |
|-----------|-----|--------|
| **yaw control** (psi+gain·bearing) | หมุนไม่หยุด (spin) | yaw_ref อิง psi สด → error คงที่ → หมุนตลอด |
| yaw held-target | ยัง spin/tumble ในจอจริง | closed-loop + vision lag (0.2s) ไม่เสถียร |
| **velocity damping** (-K_DAMP·v) | tumble (เอียงจนคว่ำ) | K_DAMP·v > look-ahead → setpoint เด้งหลังโดรน |
| supervisor + dwell ซ้อนกัน | ยิ่งซับซ้อนยิ่งพัง | interaction หลายตัว |

**บทเรียนใหญ่:**
1. **unit-test ฟังก์ชัน ≠ closed-loop** — MCP ที่ใช้ test ไม่มี `parrotlib` รันได้แค่ฟังก์ชันโดดๆ → "ผ่าน test แต่พังในจอ" ตลอด → **ต้องรัน sim จริงในจอเท่านั้นถึงเชื่อได้**
2. **yaw = ตัวขยายปัญหา** — detector ลังเลนิดเดียว + yaw = ปั่น; ไม่มี yaw = แค่ส่ายนิดๆ → ใช้ **holonomic crab** (บินเฉียง ไม่หมุนหัว) นิ่งกว่ามาก
3. **เพิ่มทีละอย่าง + รันยืนยันทุกครั้ง** (bisection) — กองรวมหลายอย่างแล้วพังจะหาตัวการไม่เจอ
4. ปัญหามุม = **detector ต้องเลือกแขนเดียวเด็ดขาด** (ไม่ใช่แก้ที่ controller/yaw)

**ทิศทางถ้าจะไปต่อ:** ถ้า crab ยัง orbit ที่มุม → ทำ detector **connected-component** (flood-fill เกาะแขนที่โดรนแตะอยู่จริง = decisive สุด) หรือไปแนว image-100% (เหมือน repo koraykzly: vision ฉลาด control โง่)

---

*จบคู่มือ — ทุก knob มีตำแหน่งไฟล์กำกับ (§8) · ก่อนเพิ่ม yaw/damping กลับ อ่าน §11 ก่อน*
