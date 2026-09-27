# R2B4 P0 — javasolt élő bizonyítási terv

## 0. Előfeltétel

```bash
cd /home/alba/project_r2b4
./r test full
./r voice restart
./r voice check
```

Először csak motormentes teszteket végezz.

## 1. Wake nem indít V3-at — kötelező P0 bizonyíték

```bash
./r rt stop
./r rt status
./r voice restart
```

Mondd:

```text
robot
```

Elvárt:

- válasz: `figyelek`;
- `./r rt status` szerint V3 továbbra sem fut;
- `runtime/wake_status.json`: `session_active=true`;
- HRI journalban `WAKE_DETECTED` és `VOICE_SESSION_OPEN`.

Ez a legfontosabb P0 acceptance.

## 2. Normál beszélgetés V3 nélkül

V3 maradjon leállítva.

Mondd:

```text
robot
mennyi kettő meg kettő?
```

Elvárt:

- Gemini válasz + Piper TTS;
- V3 nem indul el;
- `last_execution_mode=CONVERSATION`.

## 3. 10 másodperces session lifecycle

Wake után tegyél fel egy kérdést.

- újabb kérdés 10 s-on belül, új `robot` nélkül -> válaszoljon;
- az utolsó interaction után várj 10 s-nál többet;
- új wake szó nélkül beszélj -> ne nyisson új turnt;
- mondd `robot` -> újra `figyelek`.

Elvárt journal: `VOICE_SESSION_CLOSED`, reason=`SILENCE_TIMEOUT`.

## 4. Kamera V3 nélkül

```bash
./r rt stop
rm -f /tmp/r2b4_p0_camera.jpg
./r cam photo /tmp/r2b4_p0_camera.jpg
ls -lh /tmp/r2b4_p0_camera.jpg
./r rt status
```

Elvárt:

- nem üres JPEG létrejön;
- V3 nem indul el;
- a kamera a transient canonical calibrated owner útján működik.

Voice ellenőrzés:

```text
robot
készíts egy kameraképet
```

Elvárt:

- `last_execution_mode=OBSERVATION`;
- válasz: `Kameraképet készítettem.`;
- V3 továbbra sem fut.

P0-ban ez még nem képtartalom-értelmezés.

## 5. Dual camera ownership kizárása

Motor nélkül:

```bash
./r rt start
./r cam photo /tmp/should_not_exist.jpg
```

Elvárt: a fotókérés elutasított/unavailable, mert a kamerát a V3 resident vision owner birtokolja.

Utána:

```bash
./r rt stop
```

Ez bizonyítja, hogy nincs párhuzamos Picamera2 owner.

## 6. Robot-action auto-start — fizikai mozgással, külön engedélyezett teszt

Ezt csak akkor futtasd, amikor tényleges robotmozgást akarsz engedélyezni.

Kezdés:

```bash
./r rt stop
./r voice restart
```

Mondd:

```text
robot
menj előre lassan
```

Rövid mozgás után mondd:

```text
állj
```

Elvárt:

- wake után még nincs V3;
- csak a robot-action felismerésekor indul V3;
- a motorparancs a canonical V3 command útvonalon jelenik meg;
- az exact STOP canonical úton megállítja;
- voice session és V3 lifecycle egymástól független.

A tesztet csak szabad térben, alacsony sebességgel érdemes futtatni.

## 7. STOP interrupt lane

Az előző mozgásteszt során külön bizonyítható:

- action/THINKING/TTS közben mondd az exact STOP kifejezést (`állj`);
- elvárt: STOP nem várja meg a normál LLM beszélgetési forduló végét.

## Acceptance

P0 élő PASS, ha egyszerre igaz:

1. wake -> `figyelek`, V3 OFF;
2. normál Gemini beszélgetés -> V3 OFF;
3. 10 s silence timeout működik;
4. camera.photo -> V3 OFF és kalibrált JPEG;
5. V3 ON alatt nincs második kamera owner;
6. robot action -> V3 csak ekkor indul;
7. STOP canonical és gyors;
8. voice session nem a V3 start/stop állapotából következik.
