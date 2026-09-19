R2B4 P0-P1 behavior-first test upgrade

A csomag kizárólag teszteket módosít/takarít. Production V3 fájlhoz nem nyúl.

P0:
- törli a 3 véletlen _fixed.py tesztmásolatot;
- a FOLLOW_PERSON lost-target tesztet az új bounded recovery viselkedéshez igazítja;
- az L6 obstacle/local-escape teszteket a kiválasztott biztonságos mozgásra állítja át.

P1:
- Test Hub Quality wiring: source-string keresés helyett tényleges run_default() evidence-output;
- FOLLOW_PERSON: belső target-state helyett megfigyelhető fordulási/motion viselkedés;
- konkrét tuningértékek helyett relatív safety/motion invariánsok;
- L6 trajectory: candidate ID/ranking részletek helyett selected safe motion;
- checkpoint-state csak a checkpoint/restore contract tesztben marad elsődleges.

Installer:
- NINCS precondition ellenőrzés.
- NINCS SHA ellenőrzés.
- NINCS source/layout ellenőrzés.
- NINCS utólagos külön validation.
- Csak módosít/töröl, célzott pytestet futtat, majd kéri a full pytestet.

Telepítés:
1. Csomagold ki.
2. A kitömörített tartalmat másold:
   /home/alba/project_r2b4/integralando/
3. Futtasd:
   cd /home/alba/project_r2b4/integralando
   python3 installer.py
