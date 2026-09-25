R2B4 pytest refactor V5 - staging/project-root fix - 2026-09-25
================================================================

A HIBA OKA
----------
A V4 a kivalasztott teszteket tests/core, tests/feature, tests/deep ala teszi.
Tobb regi teszt azonban a sajat fajlhelyebol szamolja a repo gyokeret ezzel a
mintaval:

    Path(__file__).resolve().parents[1]

Ez flat tests/test_x.py helyzetben a repo gyokere volt. Nested
 tests/core/test_x.py esetben mar a tests/ konyvtar. Ezert a stagingben a
production configot tevesen itt kereste:

    /tmp/.../tests/conf/hardver.json

nem pedig a kanonikus repo conf/ alatt.

A config refaktor/upgrade ezt felszinre hozta, mert az uj ConfigResolver a
kanonikus 4 production dokumentumot olvassa, de a konkret hiba nem a JSON
config tartalma: a pytest staging rossz project rootot adott.

V5 JAVITAS
----------
- ellenorzi a kanonikus conf/{hardver,fizika,speed_map,vezerles}.json fajlokat;
- ideiglenesen normalizalja a legacy __file__-alapu repo-root kifejezeseket;
- staging alatt R2B4_ROOT=/home/alba/project_r2b4 autoritast ad;
- install utan R2B4_ROOT NELKUL ujra lefuttatja a teljes kuralt suite-ot;
- --check utan byte-pontosan visszaallitja az eredeti tesztfajlokat;
- install hiba eseten teljes tests/ rollback;
- production configot es production robot-control source-ot nem modosit.

HASZNALAT
---------
A mar meglevo installer_v4.py maradjon az integralando konyvtarban.

  cd /home/alba/project_r2b4/integralando
  python3 installer_v5.py --check

Ha V5 CHECK PASS:

  python3 installer_v5.py

Utana:

  cd /home/alba/project_r2b4
  ./r test
  ./r test full
