R2B4 pytest refactor V4 hotfix - 2026-09-25

MIERT KELL V4?
A V3 source-first szurese utan a valodi repo ezt adta:
  CORE=25, FEATURE=55, DEEP=3, TOTAL=83, files=16
A TOTAL mar a kivant 80-140 tartomanyban van, CORE es FEATURE is jo.
A V2 csak azert utasitotta el, mert a DEEP 20-as also hatarat hard limitnek vette.

V4 SZABALY
Hard:
- CORE >= 20
- FEATURE >= 40
- TOTAL 80..140
- max 20 tesztfajl
- staging/pytest 0 failure

Soft target:
- DEEP 20..40

A V4 NEM hoz vissza obsolete vagy implementation-coupled teszteket csak azert,
hogy DEEP=20 legyen.

MEGMARAD:
- v3/pytest_profiles.py (shared Test Hub / CLI registry)

KIZARVA MARAD:
- tests/v3_config_fixtures.py
- tests/v3_validation_helpers.py, ha a fenti obsolete fixture-re epul
- az obsolete fixture teljes dependency closure-e

HASZNALAT
A regi integralando/installer.py (V2) maradjon a helyen.

  cd /home/alba/project_r2b4/integralando
  python3 r2b4_pytest_refactor_v4_20260925/installer_v4.py --check

Ha V4 CHECK PASS:

  python3 r2b4_pytest_refactor_v4_20260925/installer_v4.py
  cd /home/alba/project_r2b4
  ./r test
  ./r test full

A V4 csak ideiglenes masolatban modositja a V2 budget logikajat. Az eredeti
integralando/installer.py fajlt nem irja at.
