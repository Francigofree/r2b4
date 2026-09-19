R2B4 V3 L7 Temporal Continuity P1-A
======================================

Source-first base:
  GitHub main commit 48678c5798ecc0dd179a6aa18760dcf92353978e

Purpose:
  Reduce Room Cruise trajectory chatter without changing L6 scoring or L8-L12.

Measured basis:
  Baseline: 54 selected-candidate changes, selected dOmega p95 0.30 rad/s.
  Offline ID-HOLD 0.005: 46 changes, dOmega p95 0.15 rad/s,
  dOmega max unchanged at 0.45 rad/s, negligible progress/clearance loss.

Install:
  1. Copy the extracted package contents into:
       /home/alba/project_r2b4/integralando/
  2. From that directory run:
       python3 installer.py

Installer behavior:
  - checks preconditions ONLY for the 3 production files it modifies;
  - no full-repository SHA check;
  - creates a bounded backup of those files;
  - installs the L7 stateful ID-HOLD selector;
  - wires L7 checkpoint/restore into NativeControlComposition;
  - keeps the headless MissionNavigationComposition behavior aligned;
  - adds a focused regression test;
  - py_compile + targeted pytest;
  - automatically restores production files if install/tests fail;
  - prints the full pytest command when done.

After successful full pytest:
  r rc 12 c full
