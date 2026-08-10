# SAFE LQR PC host

`sae_pc_hil.py` is the PC-side HIL controller for the STM32 firmware in this
repository.

## Online EAGA implementation

The online optimizer now implements the same mechanisms described in the
manuscript:

- two independently evaluated subpopulations of 10 candidates each;
- a shared, bounded elite archive containing the six best unique candidates;
- rank-dependent diffusion
  `sigma_i = sigma_base * (1 + alpha * rank_i / population_size)`;
- elite-archive-guided offspring
  `child = parent + beta * (archive_elite - parent) + N(0, sigma_i^2 I)`;
- warm starting, per-subpopulation elitism, and clipping to a bounded local
  log-weight search domain around the incumbent controller.

Candidate fitness remains the model-rollout IAE plus the saturation penalty,
and every returned gain must pass the existing stability and safety checks.
The two subpopulations are evaluated concurrently with two host threads.

## Safety-supervision downlink

Accepted candidates use the scored-controller frame below. The firmware keeps
the legacy `0xA5` parser for monitoring compatibility, but legacy frames do not
contain safety metadata and are therefore rejected by the scored pool.

```text
[AA 55 A6]
[k1 k2 k3 k4 tau_ff]                 5 x float32
[fitness]                            1 x float32
[P00 P01 P02 P03 P11 P12 P13
 P22 P23 P33]                       10 x float32
[uint8 additive checksum]           total: 68 bytes
```

After a valid candidate is processed, the firmware returns a 16-byte status
frame so the host does not treat a successful `write()` call as a controller
switch:

```text
[AA 55 5D]
[status reason frame_id]             uint8 uint8 uint16
[fitness rollback_count]             float32 uint32
[uint8 additive checksum]            total: 16 bytes
```

Status values are `1=accepted`, `2=rejected`, and `3=rollback`. The host logs
`sent` first and records `switched` only after a matching status `accepted` is
received. UART receive interrupts are restarted after a HAL UART error.

The host obtains `P` from the same continuous-time Riccati solution as the
candidate gain. The firmware verifies finite gain metadata, positive
definiteness of `P`, and improvement over the worst member when its five-entry
fitness-ranked safe-controller pool is full. It then performs a 50 ms linear
transition across all four feedback gains and `tau_ff`; a
`V(x) > 1.5 V_ss` violation rolls back to the best stored controller. Feedback
action is temporarily multiplied by `0.9` when the raw command exceeds 80% of
the actuator limit.

## Requirements

```powershell
python -m pip install numpy scipy pyserial
```

## Run

Set `MODE` near the top of `sae_pc_hil.py`:

- `selftest`: run the local simulation without hardware.
- `identify`: collect pulse-response data and identify model parameters.
- `real`: communicate with the STM32 over the configured serial port.

Then run:

```powershell
python .\PC\sae_pc_hil.py
```

For hardware use, configure `COM_PORT` first. The default is observation-only
(`SAE_ALLOW_DOWNLINK` unset or `0`). After the model, gain signs, limits, and
firmware protocol have been verified, enable the frozen experiment configuration
explicitly without editing source code:

```powershell
$env:SAE_ALLOW_DOWNLINK = "1"
python .\PC\sae_pc_hil.py
```

Record the command and commit hash with the experiment log. Unset the variable
or set it to `0` to return to observation-only mode.

## Test

Run the deterministic structural checks from the repository root:

```powershell
python -m unittest PC.test_eaga
```

The tests verify the two-subpopulation layout, elite-archive capacity and
ordering, monotonically rank-scaled diffusion, finite optimizer output, and a
68-byte scored-controller frame round trip including checksum and symmetric
Riccati-matrix reconstruction.
