# R2B4 — legacy retirement

A robotarchitektúra authority: `STRUKTURALIS_RETEGEK_V3.md`. Az aktuális production source és aktív config az elsődleges.

Feladat: tisztítsd a repót V3-only állapotra. A legacy-t töröld, ne archiváld és ne építs compatibility/donor réteget.

Dolgozz közvetlenül a Git working tree-ben. Ne használj candidate-, promotion-, lease-, claim-, receipt-, capsule- vagy külön evidence-admin workflow-t.

Törlés előtt ellenőrizd a túlélő V3 hivatkozásokat. Legacy-függő teszteket alakíts natív V3 contract/characterization tesztekké, ne konzerváld a legacy implementációt fixture-ként.

Ne változtass robotikai viselkedést vagy runtime safety-t. Ne nevezd át és ne szervezd át most a V3 production struktúrát.

A végén futtasd a szükséges V3-only teszteket, import guardot és meglévő natív capture replayt. Live motorfutás nem szükséges.

Jelentsd röviden: mi törlődött, mi maradt, validáció, fennmaradó probléma.
