# Single-Phase Grid-Connected MMC — Arm Leaders & Energy Balancing

**MATLAB/Simulink model | 8 half-bridge submodules per arm | R2024a**

> This repository preserves the original single-phase grid-connected MMC Simulink model and provides concise English documentation of the arm-leader control, energy balancing, neighbor-consensus logic, and PS-PWM switching. The original `.slx` file is unchanged.

## 1. What is included?

- **Power circuit:** one-phase grid-connected MMC, 8 HB submodules in each arm (16 total), arm inductors, AC line/filter and 230 V RMS / 50 Hz grid source.
- **Outer loops:** grid-voltage SOGI-PLL, active/reactive power measurement and P/Q-to-current-reference regulation (one identical instance per arm leader).
- **Inner arm loops:** current PI + resonant compensation, DC/circulating-current and arm-energy regulation; includes an enabled 100 Hz circulating-current suppression term.
- **Inter-arm exchange:** each leader sends **one scalar arm-energy value** to the other through a one-control-sample delay.
- **Local SM switching:** each submodule uses its voltage plus the two adjacent ring-neighbor voltages to adjust its modulation duty and generate complementary half-bridge gate signals with PS-PWM.

**Important scope note:** The model *does* implement closed-loop control. Ring-neighbor communication is emulated inside Simulink by reading the arm-voltage vector; it is **not** a verified hardware communication implementation. No simulation/experimental performance result is claimed by this repository.

## 2. Repository layout

```text
MMC_1ph_Grid_ArmLeaders_EnergyBalancing/
├── README.md
├── model/
│   └── MMC_1ph_Grid_ArmLeaders_EnergyBalancing_v3_1A.slx
├── reference_code/
│   ├── mmc_outer_ctrl.m
│   ├── mmc_arm_ctrl.m
│   ├── hb_sm_local_ctrl.m
│   ├── get_neighbors.m
│   └── meas_avg.m
├── matlab/
│   └── run_model.m
└── docs/
    ├── ARCHITECTURE.md
    └── VALIDATION_CHECKLIST.md
```

## 3. Quick start

**Requirements:** MATLAB + Simulink **R2024a** (the model's save release), and **Simscape Electrical — Specialized Power Systems** blocks. A compatible later MATLAB version may work but has not been tested here.

1. Open MATLAB and set the Current Folder to the repository root.
2. Open `model/MMC_1ph_Grid_ArmLeaders_EnergyBalancing_v3_1A.slx`.
3. The model's embedded `PreLoadFcn` / `InitFcn` initializes its parameters automatically.
4. Click **Run** (model Stop Time: **1 s**) and examine the scopes below.

Or execute from MATLAB:

```matlab
run('matlab/run_model.m')
```

### Useful Scopes

- `AC output v_ac i_ac`
- `P and Q measured W var`
- `Arm currents i_u i_l`
- `SM voltages UPPER` and `SM voltages LOWER`
- `Modulation indices`
- `UP W_own_f W_other_f`, `UP i_ref i_circ_ref`
- `i_circ = (i_u + i_l) over 2`

## 4. Baseline parameters (as embedded in the model)

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `VDC` | 800 V | DC-link voltage |
| `N` | 8 / arm | 16 half-bridge SMs total |
| `C` | 2.2 mF | SM capacitance |
| `vo` | 100 V | Nominal SM capacitor voltage |
| `Larm`, `Rarm` | 5 mH, 0.5 Ω | Each arm |
| `Vg_rms`, `f` | 230 V, 50 Hz | Grid |
| `Lg`, `Rg` | 2 mH, 0.1 Ω | AC line |
| `Pstar`, `Qstar` | 1500 W, 0 var | Power references |
| `fsw` | 1250 Hz | PS-PWM carrier for each SM |
| `Ts` | 10 µs | Electrical simulation / local switching |
| `Ts_ctrl` | 100 µs | Arm-leader control |
| `Cf`, `Rf` | 10 µF, 10 Ω | AC filter |

**Caution:** some controller parameters are also **hard-coded inside MATLAB Function blocks** (`N=8`, `fsw=1250`, `Ts=1e-4`, `C=2.2e-3` and controller gains). Changing only the model callback parameters will **not** update every internal controller constant. Review `reference_code/` and the actual blocks before modifying the design.

## 5. Documentation

- [Architecture & signal flow](docs/ARCHITECTURE.md)
- [Validation checklist](docs/VALIDATION_CHECKLIST.md)

`reference_code/*.m` is a **read-only documentation snapshot** extracted from the model's embedded MATLAB Function blocks. **Edits to these `.m` files do not modify the `.slx` model.** The original SLX remains the authoritative source.

## 6. Verification and licensing status

- **Verified:** original `.slx` was inspected as an SLX archive, and its controller scripts / topology / parameters were extracted for documentation.
- **Not verified:** opening, compiling or simulating the model in MATLAB, numerical tracking, control stability, real-time execution or physical hardware validation.
- **License:** none assigned. Keep the repository **private** until the rights of all contributors/model components and the desired license are confirmed. MATLAB/Simscape products retain their own licenses.
