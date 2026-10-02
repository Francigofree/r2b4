# R2B4 Gemini structured-output P0 fix

Verified against `Francigofree/r2b4` commit:

`e06f443d91ece6c34de3bcef0cd9ff976fe4e8e6`

## Problem fixed

`GeminiStructuredChatClient` used the Interactions API with `gemini-2.5-flash` and
expected `response_format` to force JSON. Live testing showed a successful
`status=completed` interaction returning plain text, so `json.loads()` failed before
the first Agent Core tool request.

The fix keeps the working plain `r -- "..."` path on Interactions, but moves only
structured conversation/Agent Core calls to the Gemini `generateContent` endpoint.
It sends:

- `generationConfig.responseMimeType = application/json`
- `generationConfig.responseJsonSchema = <existing R2B4 schema>`
- Gemini 2.5 thinking control via `thinkingConfig.thinkingBudget`

Agent Core, routing, tool contracts, robot authority and V3 paths are unchanged.

## Apply

From repository root:

```bash
python3 integralando/apply_r2b4_gemini_structured_fix.py --root . --check-only
python3 integralando/apply_r2b4_gemini_structured_fix.py --root . --run-tests
```

If HEAD has moved but `r2b4_voice/gemini_llm.py` is still exactly the verified
blob, use `--allow-head-mismatch`.

## Live validation

```bash
python3 integralando/live_smoke_gemini_agent.py
r "Miért kék az ég? Egy rövid mondatban válaszolj."
r "Source-first: keresd meg a saját forráskódodban, hol van az exact STOP fast-path. Ne mozogj és ne használj ER2-t. Add meg a fájlt és a releváns függvényeket."
```

The first command must print `PASS`. The second must produce a normal Agent answer.
The source-first command should then create `agent_tool_requested/completed` journal events.
