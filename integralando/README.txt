R2B4 — ChatGPT OAuth + automatic LLM failover upgrade
2026-10-02

CÉL
- ChatGPT/OpenAI legyen az elsődleges LLM.
- Elsődleges hitelesítés: Sign in with ChatGPT OAuth, API key nélkül.
- Mentett access token automatikus használata.
- Lejáró access token automatikus refresh-e, rotating refresh token mentéssel.
- OPENAI_API_KEY csak opcionális fallback.
- Gemini fallback megmarad.
- Groq LLM fallback is megmarad, ha GROQ_API_KEY van.
- Kvóta/rate-limit/hibás válasz/provider hiba esetén automatikus provider-váltás.
- AgentCore / RobotInterface / V3 safety authority változatlan.

FORRÁS-ÁLLAPOT
A csomag a Francigofree/r2b4 jelenlegi main szerkezetére és az előző
r2b4_chatgpt_primary_upgrade_20261002 csomaggal már frissített állapotra is
felkészült. A telepítő nem igényel git/network ellenőrzést.

TELEPÍTÉS A PI-N

  unzip r2b4_chatgpt_oauth_failover_upgrade_20261002.zip
  cd r2b4_chatgpt_oauth_failover_upgrade_20261002
  python3 install.py /home/alba/project_r2b4

A telepítő:
- egy timestampelt backupot készít .upgrade_backups/ alatt;
- létrehozza/preserválja a conf/.chatgpt_host_id fájlt 0600 móddal;
- nem törli a meglévő conf/.chatgpt_oauth.json credentialt;
- beállítja:
    R2B4_LLM_PROVIDER=openai
    R2B4_LLM_MODEL=gpt-5.6
    R2B4_LLM_FAILOVER=1
- nem indít robot runtime-ot.

CHATGPT OAUTH — HA A BÖNGÉSZŐ UGYANAZON A GÉPEN FUT

  cd /home/alba/project_r2b4
  ./r chatgpt login

A callback 127.0.0.1-re érkezik. A sikeres credential ide kerül:

  /home/alba/project_r2b4/conf/.chatgpt_oauth.json

A fájl owner-only (0600), nem kerülhet gitbe/logba.

HEADLESS PI — JAVASOLT MENET
A 127.0.0.1 callback mindig a böngészőt futtató gépre mutat, ezért távoli/headless
Pi esetén a böngészős OAuth-t a PC-n kell befejezni, de a Pi saját host-ID-jával.

1. Pi:

  cd /home/alba/project_r2b4
  ./r chatgpt host-id

2. A kicsomagolt upgrade csomagban, a böngészős PC-n (Windows/Linux/macOS):

  python chatgpt_login.py --host-id "IDE_MÁSOLOD_A_PI_HOST_ID_T" --output r2b4_chatgpt_oauth.json

3. Másold át a létrejött r2b4_chatgpt_oauth.json fájlt biztonságosan a Pi-re.

4. Pi:

  cd /home/alba/project_r2b4
  ./r chatgpt import /UTVONAL/r2b4_chatgpt_oauth.json
  ./r chatgpt status

5. Az átmásolt ideiglenes credential fájlt töröld; az R2B4 saját védett példánya a
   conf/.chatgpt_oauth.json.

TOKEN KEZELÉS
- Érvényes mentett access token: automatikusan használja.
- Lejárat közelében: automatikus refresh.
- A refresh művelet processzek között lockolt, mert a refresh token rotating.
- Sikeres refresh után az access + replacement refresh token együtt, atomikusan
  kerül mentésre.
- Ha a refresh már nem használható: az OpenAI OAuth ág kiesik és a failover lánc
  következő konfigurált tagja használható; új OAuth-hoz: ./r chatgpt login.

LLM FAILOVER SORREND ALAPÉRTELMEZETTEN

  1. OpenAI / ChatGPT OAuth
  2. OpenAI API key              (ha OPENAI_API_KEY létezik)
  3. Gemini                      (ha GEMINI_API_KEY / GOOGLE_API_KEY létezik)
  4. Groq                        (ha GROQ_API_KEY létezik)

A provider-specifikus fallback modellek nem öröklik vakon a primary
R2B4_LLM_MODEL értéket. Opcionális modellek:

  R2B4_OPENAI_MODEL=...
  R2B4_GEMINI_MODEL=...
  R2B4_GROQ_MODEL=...

OAuth esetén az adapter megpróbálja lekérni a bejelentkezett ChatGPT account
látható /v1/models katalógusát. Ha a preferált OpenAI modell elérhető, azt
használja; ha nem, a szerver által elsőként listázott látható modellt. Ha maga a
modellista nem érhető el, az inference még megpróbálkozik a konfigurált modellel.

HIBAPOLITIKA
- 429 / quota / rate-limit: nincs azonos-provider retry; azonnal következő LLM.
- ChatGPT-plan usage-limit/unavailable: azonnal következő LLM.
- Strukturált/érvénytelen/üres LLM-válasz: azonnal következő LLM.
- Átmeneti hálózati vagy tipikus 5xx hiba: maximum 1 rövid újrapróbálás, utána
  következő LLM.
- Hibázó provider rövid cooldownra kerül a rezidens processzben.
- Ha minden konfigurált provider elfogyott/hibás, fail-closed LLM error lesz.

A failover egy LLM-hívást vált át másik providerre. Már lefutott AgentCore toolt
vagy robotakciót nem ismétel meg, és nem változtatja meg a canonical R2B4
RobotInterface/V3 authority útvonalat.

API KEY FALLBACK — OPCIONÁLIS
Ha akarod:

  OPENAI_API_KEY=...
  GEMINI_API_KEY=...
  GROQ_API_KEY=...

mehet a /home/alba/project_r2b4/conf/.wake.env fájlba. OPENAI_API_KEY nem kell az
OAuth működéséhez.

FONTOS: a jelenlegi Voice STT továbbra is Groq STT. Tehát voice inputhoz a
GROQ_API_KEY jelenleg továbbra is szükséges, függetlenül attól, hogy az LLM
ChatGPT OAuth-pal működik-e.

ELLENŐRZÉS

  cd /home/alba/project_r2b4
  ./r chatgpt status
  python3 -m r2b4_voice.voice_service --check
  python3 -m pytest -q \
    tests/core/test_openai_oauth.py \
    tests/core/test_llm_failover.py \
    tests/core/test_openai_responses_transport.py \
    tests/core/test_openai_primary_provider.py

Ezután, ha a voice user service fut:

  systemctl --user restart r2b4-wake.service

CHATGPT OAUTH PARANCSOK

  ./r chatgpt status
  ./r chatgpt host-id
  ./r chatgpt login
  ./r chatgpt import FILE
  ./r chatgpt logout

A logout megpróbálja visszavonni a renewable sessiont, majd lokálisan törli a
használható tokeneket, de megtartja a kliens/account mappinget és a Pi host-ID-t.
