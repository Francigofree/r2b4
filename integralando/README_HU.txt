R2B4 PYTEST REFAKTOR V6 - FALSE ROLLBACK FIX
============================================

A valodi V5 install log szerint a kuralt suite sikeresen telepult:
- staging 83/83 PASS
- ./r test 25/25 PASS
- ./r test full 83/83 PASS

A rollback oka a V4 sajat utovalidatora volt. Az AST-ban 70 darab test_* Python
fuggvenyt szamolt, mikozben pytest parametrizacioval 83 tesztesetet collectalt.
Ezert a jo 83-as suite-ot tevesen 70-esnek minositette.

V6 javitas:
- V4 AST TOTAL also limitet csak ideiglenes masolatban kikapcsolja;
- a 80..140 hard budgetet pytest --collect-only item-szammal ellenorzi;
- CORE >=20, FEATURE >=40, <=20 fajl hard guard marad;
- DEEP 20..40 soft target marad;
- teljes pytest, ./r test es ./r test full kotelezo PASS;
- install elott sajat rollback snapshot keszul;
- a sikertelen V5 utan esetleg megmaradt r / v3/test_runner.py allapotot a legfrissebb V2 backupbol helyreallitja, ha ott elerheto.

HASZNALAT
---------
A mar kicsomagolt installer_v4.py es installer_v5.py maradjon az integralando
konyvtarban.

  cd /home/alba/project_r2b4/integralando
  python3 r2b4_pytest_refactor_v6_20260925/installer_v6.py --check

Ha V6 CHECK PASS:

  python3 r2b4_pytest_refactor_v6_20260925/installer_v6.py

Majd:

  cd /home/alba/project_r2b4
  ./r test
  ./r test full
