# R2B4 Gemini structured-output fix v2

Base repository: `Francigofree/r2b4`
Verified base commit: `e06f443d91ece6c34de3bcef0cd9ff976fe4e8e6`

This is the corrected package for the Gemini 2.5 structured-output transport fix.

## What is fixed

- Structured Gemini turns use `generateContent` + `responseMimeType=application/json` + `responseJsonSchema`.
- Plain `r -- "..."` Gemini remains on the existing Interactions API.
- Gemini 2.5 thinking uses `thinkingBudget`.
- Structured parse failures include a bounded output prefix for diagnostics.
- v2 fixes the unit-test mock signature so it accepts the production client's `timeout=` keyword argument.

The failed v1 `--run-tests` application rolls production files back automatically, so v2 can be applied normally afterwards.

## Apply

From the repository root:

```bash
python3 integralando/apply_r2b4_gemini_structured_fix.py --root . --check-only
python3 integralando/apply_r2b4_gemini_structured_fix.py --root . --run-tests
```

Then live smoke:

```bash
r "Miért kék az ég? Egy rövid mondatban válaszolj."
r "Source-first: keresd meg a saját forráskódodban, hol van az exact STOP fast-path. Ne mozogj és ne használj ER2-t. Add meg a fájlt és a releváns függvényeket."
```
