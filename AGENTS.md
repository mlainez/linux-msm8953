# Contributing to this msm8953 kernel fork (humans and AI agents)

This is a downstream Linux kernel fork for **Qualcomm MSM8953 / SDM632**
devices — primarily the **Fairphone 3 and 3+**. It tracks
[msm8953-mainline/linux](https://github.com/msm8953-mainline/linux) and the
mainline LTS stable branches, and owns its patches.

Its consumer is **`nerves_system_fp3`** (Buildroot + Nerves, headless Elixir
images). That system pins this fork by commit SHA and builds it with
Buildroot; there is no Debian packaging in this family.

Read this file before creating branches or commits.

---

## 0. The one thing to understand first

**The Fairphone 3 and 3+ are the same mainboard.** Both report
`qcom,msm-id = <349 0>` and `qcom,board-id = <8 0x10000>`. What differs is
which *modules* are fitted, and every one of them is user-replaceable with a
#00 screwdriver and sold separately:

| Slot | Fairphone 3 | Fairphone 3+ |
| --- | --- | --- |
| Rear camera | Sony IMX363 (12 MP) | Samsung S5KGM1SP (48 MP) |
| Rear autofocus | AK7374 | DW9800W |
| Front camera | Samsung S5K4H7YX (8 MP) | Samsung S5K3P9SP (16 MP) |
| Loudspeaker | Awinic AW8898 | TI TAS2557 |

A given phone can carry **any mix** of these. Real hardware seen in the lab
includes an FP3 chassis with FP3+ camera modules. There is therefore no
"FP3 device tree" and "FP3+ device tree" to choose between — the variant is
not a property of the board.

So `sdm632-fairphone-fp3.dts` describes the **slots**: which I²C bus each
module hangs off, its regulators, its reference clock, its enable line.
`drivers/misc/fp3_module_slot.c` powers each slot at boot, reads the chip's
ID register, and applies a device tree overlay describing what it found
(the in-kernel `of_overlay_fdt_apply()` path, as in
`drivers/misc/lan966x_pci.c` — *not* the out-of-tree configfs interface).
Stock sensor and codec drivers then bind against a tree that finally
describes hardware that is actually present.

Consequences that catch people out:

- **There is exactly one DTB.** Do not add a second board DTS for a variant.
  `sdm632-fairphone-fp3p.dts` was deleted deliberately.
- **Addresses are not identities.** Both front sensors answer at `0x10`; the
  rear IMX363 has been observed at both `0x1a` (what mainline assumed) and
  `0x10` (two of two lab units). Identify by ID register, never by address.
- **The base DTB must be built with `-@`** so overlays can resolve their
  targets through `/__symbols__`. See `DTC_FLAGS_sdm632-fairphone-fp3` in
  `arch/arm64/boot/dts/qcom/Makefile`.
- **CAMSS and the sound card are `status = "disabled"` in the base tree.**
  They are enabled by overlay once their inputs exist.
  `of_reconfig_get_state_change()` turns a `disabled → okay` flip into an
  `OF_RECONFIG_CHANGE_ADD`, which is what creates the platform device.

---

## 1. Branching strategy

Names are **version-first**, slash-namespaced. The SoC (`msm8953`) is the
whole fork, so it is not repeated in branch names; the kernel version is the
top axis because several may be maintained in parallel.

```
<kver>/topic/<feature>   →   <kver>/staging   →   <kver>/rc   →   <kver>/release
       (one feature)           (integrate all)     (candidate)      (blessed)
```

| Tier | Example | Purpose | Lifecycle |
| --- | --- | --- | --- |
| **base** | `<kver>/baseline` | stable `linux-<kver>.y` plus msm8953-mainline's work for that series | tracked; never committed to. Topics pin a tested base commit; advancing it is a deliberate, re-tested step. |
| **topic** | `6.19/topic/module-slots`, `6.19/topic/camera-support`, `6.19/topic/pmi632`, `6.19/topic/slimbus-audio` | exactly one feature/fix-set; clean, rebasable, **upstreamable** | long-lived; rebased onto base |
| **staging** | `6.19/staging` | merge **all** topics; flashed to devices for combined testing | fast-moving; may be reset/rebuilt |
| **rc** | `6.19/rc` | release candidate — what `nerves_system_fp3` pins for pre-release testing | promoted from staging when it passes on-device validation (§1.1) |
| **release** | `6.19/release` | blessed; what shipped systems pin | fast-forwarded from rc when validated; tag releases here |

Rules:

- **A topic branch holds one feature and nothing else.** Keep it clean enough
  to submit upstream. No integration merges, no other topics, no device-only
  hacks.
- **Do not commit directly to `staging`/`rc`/`release`.** They are built by
  `git merge --no-ff <kver>/topic/*`. The only non-topic commits allowed on
  integration branches are fork meta (this file, CI workflows) and automated
  stable merges.
- **Device variants are not branches, and on this SoC they are not DTBs
  either** — they are runtime-detected modules (§0). Never fork a branch or
  add a DTS to toggle a board option.
- **`nerves_system_fp3` pins a commit SHA**, not a branch name. Repoint
  `BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION` when promoting.

### 1.1 Promotion gates

Promotion is evidence-driven; each tier has a gate that must pass *on the
device* before moving up.

1. **fix/feature → topic**: compiles warning-free for its own files with the
   aarch64 cross toolchain (`make W=1 drivers/<sub>/`); one logical change per
   commit (bisectable).
2. **topic → staging**: `git merge --no-ff`, then build a full image from
   staging and validate on-device. A topic is not "in" until the *merged*
   image passed — clock, genpd and probe-ordering bugs only appear in
   combination.
3. **staging → rc**: promote only after the full sweep passes:
   - the feature demonstrably works, not merely probes (a camera that
     enumerates `/dev/video*` and captures a frame; a sound card that appears
     in `/proc/asound/cards`, not just an amplifier that ACKs);
   - **module-matrix check**: boot the same image on an FP3 *and* an FP3+ and
     confirm all three slots identify correctly on both. This is the fork's
     defining feature and the easiest thing to regress;
   - **regression sweep**: modem/WCNSS/ADSP remoteprocs up, QRTR clean,
     network, GPU submit, NFC, GNSS;
   - **stability soak** at idle before promoting.
4. **rc → release**: human decision after wider testing; tag on release.

If a gate fails, fix it *on the topic branch* and re-merge; staging may be
rebuilt. `rc`/`release` never receive unvalidated work.

### 1.2 Device-testing protocol (hard-won rules)

- **Flash `userdata` only.** The Nerves image is a full disk image nested
  inside the Android `userdata` partition. `fastboot flash userdata
  <app>.img` is the only flash command that should ever be issued to these
  phones. Do not touch `boot` (lk2nd lives there), or any other partition.
- **Get to fastboot without touching the phone**: `/usr/sbin/reboot-mode
  bootloader` over SSH. It does not return, so invoke it detached and then
  watch `fastboot devices` from a *separate* command — never both in one
  shell invocation.
- **Reaching a booted device**: the Nerves image brings up a USB gadget with
  `VintageNetDirect`. Two phones derive the *same* `172.31.x.y/30` and
  collide, so address them by scoped IPv6 link-local instead:
  `ping6 -c2 -I <iface> ff02::1` then `ssh fe80::…%<iface>`. Beware that the
  host's own address on that link answers SSH too — if the banner says
  `OpenSSH`, you are talking to your laptop, not the phone (nerves_ssh is
  Erlang's SSH).
- **Verify the image before flashing.** Buildroot and Nerves will both hand
  you a stale kernel without saying so (§4). Check `md5sum` of
  `images/Image` against the tree you just built, and confirm a string you
  just added is present in `vmlinux`.
- **Judge by evidence, not by absence of errors.** A slot that reports
  "no known module found" should be made to say *why*; the driver prints
  per-candidate results and a full bus scan with ID registers for exactly
  this reason. That diagnostic is what identified an IMX363 at an
  unexpected address in one boot instead of a rebuild cycle per guess.

---

## 2. Commit conventions

Follow the upstream rules —
<https://www.kernel.org/doc/html/latest/process/submitting-patches.html>:

- **Subject:** `subsystem: imperative summary`, ≤ ~75 chars, lower-case after
  the prefix, no trailing period. Match the prefix the subsystem already uses
  (`arm64: dts: qcom:`, `media: i2c:`, `ASoC: qcom:`, `misc:`, …).
- **Body:** wrapped at ~75 columns; explain **what and why** (and user-visible
  impact / trade-offs), not a line-by-line "how". Reference commits as
  `<12+ hex> ("oneline subject")`.
- **One logical change per commit**; each must build on its own.
- **`Fixes:`** for regressions: `Fixes: <12+ hex> ("subject")`. Add
  `Link:`/`Closes:` (prefer lore.kernel.org) when there is a report.

### Author identity and trailers (fork policy)

- **Work authored here** is committed as `Marc Lainez <marc.lainez@gmail.com>`.
- **Commits from other trees are cherry-picked, not rewritten**:
  `git cherry-pick -x` preserves the original author and records provenance.
  Do not re-author other people's patches as ours.
- **Never add a new `Signed-off-by:`.** These are fork branches, not upstream
  submissions, so no DCO is required. Trailers already present on
  cherry-picked commits stay as-is; just never *add* one.
- **No co-authorship** — never `Co-developed-by:`/`Co-authored-by:`; the AI is
  never a co-author.
- **Do add an AI-assistance disclosure** on Marc-authored commits when an
  assistant materially helped, per the kernel.org convention (§3):

  ```
  Assisted-by: Claude:claude-opus-5
  ```

  If a `Signed-off-by:` is ever needed (an actual upstream posting), that is a
  human action taken by the maintainer at submission time.

---

## 3. AI coding-assistant guidelines (kernel policy)

Per <https://www.kernel.org/doc/html/latest/process/coding-assistants.html>,
when work from this fork is submitted **upstream**:

- **Licensing:** all code must be GPL-2.0-only compatible; every new file gets
  a correct `SPDX-License-Identifier`.
- **DCO / `Signed-off-by`:** only a human can certify the DCO. **An AI agent
  must never add `Signed-off-by`.** The human submitter adds their own and
  takes full responsibility for reviewing the code, its correctness and its
  legal compliance.
- **Disclosure:** AI assistance is disclosed with `Assisted-by:`
  (agent:model, then any specialized analysis tools actually used — not
  git/gcc/make). Disclosure, not co-authorship.
- **Process:** follow the kernel coding style and `submitting-patches`. The
  assistant assists; the human is accountable for every line submitted.

---

## 3.1 Debugging walls: the authority order (hard rule)

When a hardware-facing symptom survives **two fix attempts** (or an hour
without a mechanism), stop theorising against the tree you are working in —
it is the one source known not to contain the answer — and walk the authority
order:

1. **the hardware itself** — on this fork that usually means a bus scan with
   ID registers, powered by the same sequence the real driver uses. Two
   guesses about the rear camera address cost more than the one diagnostic
   that answered it outright;
2. **vendor/downstream source** (register-level truth) — note that the
   LineageOS msm8953 kernel checkout is the *generic* tree and carries no
   Fairphone camera DT, so it is not an authority for this board;
3. **sibling ports** (msm8953-mainline, pmaports devices);
4. **this fork's own history** — branches, reverts, abandoned attempts. The
   answer to the loudspeaker I²C bus properties was already written in a
   comment on a branch;
5. **upstream history** between the last-working and current base.

Device experiments decide between authority-produced hypotheses; they do not
replace the reading.

---

## 4. Build & test (quick reference)

Built via `nerves_system_fp3` + Buildroot. Two traps, both of which have
silently shipped the wrong kernel:

- **The Nerves artifact cache does not see kernel changes.** The artifact
  checksum is computed from `package_files()` in the system's `mix.exs` —
  defconfigs and patches — not the kernel source. With
  `LINUX_OVERRIDE_SRCDIR` set in `local.mk`, editing this tree changes
  nothing the checksum can observe, so `mix deps.compile … --force`
  cheerfully re-links a stale artifact. While iterating on the kernel, run
  `make linux-rebuild` inside the artifact directory instead, then regenerate
  `boot.img` and the image. Once the system pins a SHA and `local.mk` is
  gone, the trap disappears.
- **Buildroot's git cache snapshots branch refs** and reuses stale sources.
  Pin `BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION` to a **commit SHA** for anything
  you intend to reproduce.

Useful one-liners on a booted device:

```sh
dmesg | grep fp3-module-slots      # which modules were identified
cat /proc/asound/cards             # sound card enumerated?
ls /sys/bus/i2c/devices/           # sensor/amp clients instantiated
ls /dev/video*                     # CAMSS graph completed
```
