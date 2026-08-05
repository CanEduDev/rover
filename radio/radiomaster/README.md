# RadioMaster MT12 Provisioning Guide

Provisioning guide for the **RadioMaster MT12** 2.4GHz transmitter with **Bandit Nano** 900MHz module, and **XR1 Nano Dual-Band** receiver.

The transmitter is configured with two models: **`2400MHz`** for normal operation (better performance) and **`900MHz`** for congested RF environments such as expos or crowded office spaces. To switch bands in the field, power off the Rover, select the desired model on the transmitter, then power the Rover back on — allow a few seconds for the receiver to reconnect.

---

## 1. Model Import (EdgeTX Companion)

Import the Rover model profile onto the MT12 via [EdgeTX Companion](https://edgetx.org/getedgetx/).

- [ ] Power on the MT12.
- [ ] Connect the MT12 to a PC using the **side USB-C port**.
- [ ] When prompted, select **USB Storage (SD)**.
- [ ] Open **EdgeTX Companion** on the PC.
- [ ] Go to **File → Open**, and select `Rover-profiles.etx`.
- [ ] Go to **File → Write Models and Settings to Radio**.
- [ ] Confirm the write and wait for completion.
- [ ] Safely eject the MT12 and reboot it.
- [ ] Verify both **`2400MHz`** and **`900MHz`** models appear in the model list on the MT12.

---

## 2. Transmitter Configuration (ExpressLRS WebUI)

Configure each ExpressLRS module via its built-in WiFi hotspot. Repeat this procedure **once for the 2.4 GHz module** and **once for the 900 MHz module**, using the corresponding JSON file.

| Model on MT12 | Config File |
|---|---|
| `2400MHz` | `elrs-2400mhz.json` |
| `900MHz` | `elrs-900mhz.json` |

### 2.1 Enable TX WiFi

- [ ] On the MT12, select the model corresponding to the module being configured (e.g. **`2400MHz`** or **`900MHz`**) from the model select screen.
- [ ] Navigate to the **ExpressLRS Lua script** (typically found under `SYS → Tools → ExpressLRS`).
- [ ] Select **WiFi Connectivity → Enable TX WiFi**.
- [ ] Wait for the module to broadcast its WiFi hotspot (LED will indicate WiFi mode).

### 2.2 Import Config and Set Binding Phrase

- [ ] On the PC, connect to the **`ExpressLRS TX`** WiFi network (password: `expresslrs`).
- [ ] Open a browser and navigate to `http://10.0.0.1`.
- [ ] Go to **Options**.
- [ ] Click **Browse**, then upload the appropriate JSON config file for this module.
- [ ] Set the **Binding Phrase** to the fleet-standard phrase.
- [ ] Click **Save** and wait for confirmation.
- [ ] Reboot the MT12.
- [ ] Repeat **Section 2** for the second module.

> **Note:** Each Rover must use a unique binding phrase. Reusing the same phrase across multiple Rovers will cause binding conflicts between transmitters.

---

## 3. Receiver Configuration (XR1 Nano WebUI)

Configure the **XR1 Nano Dual-Band** receiver via its WiFi interface. The receiver must be bound with the same Binding Phrase set on the transmitter modules.

### 3.1 Enter WiFi Mode

- [ ] Power on the receiver **without** the transmitter active.
- [ ] Wait **60 seconds** — the receiver will enter WiFi mode automatically after failing to bind.
- [ ] On the PC, connect to the **`ExpressLRS RX`** WiFi network (password: `expresslrs`).
- [ ] Open a browser and navigate to `http://10.0.0.1`.

### 3.2 Configure Protocol, Failsafe, and Binding Phrase

- [ ] Under **Serial Protocol**, select **SBUS**.
- [ ] Under **Failsafe Mode**, select **Last Position**.
- [ ] Set the **Binding Phrase** to the same phrase used in Section 2.
- [ ] Click **Save** and wait for confirmation.
- [ ] Restart the receiver.

### 3.3 Verify Binding

- [ ] Power on the MT12 and select the appropriate model (`2400MHz` or `900MHz`).
- [ ] Power on the receiver.
- [ ] Confirm the receiver LED indicates a **bound/connected** state.
- [ ] Arm the system by moving the SA switch to the outermost position.
- [ ] Verify control inputs are received by the Rover.
