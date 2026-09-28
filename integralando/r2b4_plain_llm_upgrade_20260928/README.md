# R2B4 plain LLM launcher upgrade

Source-first alap:

- repository: `Francigofree/r2b4`
- branch: `main`
- commit: `b44b183ed9c626b7df8fa6b6d12d5ed3fce93560`
- baseline `v3/launcher_cli.py` Git blob: `98f885727add163f21181eda005910fa88a40ab3`
- készítés dátuma: 2026-09-28

## Mit valósít meg?

Új launcher használat:

```bash
./r "Miért kék az ég?"
```

Végrehajtási út:

```text
r
 -> v3.launcher_cli
 -> r2b4_voice.plain_llm.PlainGeminiClient
 -> Gemini Interactions API (plain text, nincs LLMDecision/action schema)
 -> válasz kiírás stdout-ra
 -> meglévő build_tts_client()
 -> alapértelmezetten lokális Piper
 -> meglévő PcmWavePlayer
 -> hangszóró
```

Nem indul V3, ER2, RobotInterface, kamera, tool/action végrehajtás vagy conversation orchestration.

Az ismert `r` parancsok elsőbbséget élveznek. Ha a prompt maga egy parancsnév, explicit escape kell:

```bash
./r -- "s"
./r -- "help"
```

A hosszú válaszokat a rendszer legfeljebb 1800 karakteres TTS-darabokra bontja, így a jelenlegi 2000 karakteres Piper/Gemini TTS limitet nem lépi túl.

## Telepítés

```bash
cd r2b4_plain_llm_upgrade_20260928
python3 apply_upgrade.py --root /home/alba/project_r2b4
```

A telepítő:

1. ellenőrzi, hogy a launcher pontosan a fenti source-first baseline,
2. backupot készít `.upgrade_backups/plain_llm_<UTC timestamp>/` alá,
3. patcheli `v3/launcher_cli.py`-t,
4. telepíti `r2b4_voice/plain_llm.py`-t,
5. telepíti a célzott tesztet,
6. lefuttatja a `py_compile` ellenőrzést,
7. lefuttatja:

```bash
python3 -m pytest -q tests/feature/test_plain_llm_launcher.py
```

Ha a compile vagy a teszt hibázik, a telepítő automatikusan visszaállítja a launchert és eltávolítja az új fájlokat.

## Konfiguráció

LLM:

- provider ebben az útban: **Gemini**
- API kulcs: `GEMINI_API_KEY`, fallback: `GOOGLE_API_KEY`
- forrás: process environment, majd `conf/.wake.env`
- model: `R2B4_LLM_MODEL`, alapértelmezés: `gemini-3.5-flash-lite`

TTS:

- a meglévő `build_tts_client()` az SSOT
- alapértelmezés: `piper`
- alap hang: `hu_HU-anna-medium`
- a jelenlegi `R2B4_TTS_PROVIDER` felülbírálás továbbra is működik

## Gyors ellenőrzés

```bash
./r "Válaszolj egyetlen rövid mondatban: mennyi 23 szor 47?"
./r -- "s"
./r commands
```

Elvárt működés: a Gemini válasza először megjelenik a terminálban, majd ugyanaz a teljes szöveg megszólal a hangszórón.
