# Source-first audit notes

A refaktor a `Francigofree/r2b4` `main` ág
`dcc4c2c77928bc06b1103c949757024869a9df6b` állapotából készült.

Megállapítások a forrásból:

- A jelenlegi EVI canonical producer: `tools/mcap_evidence/`.
- Az EVI `index.sqlite` `messages`, `fields`, `identifiers`, `views` táblákat
  publikál; a normalized view-k `message_id` és `source_pointer` lineage-et
  hordoznak.
- Az EVI verifier payload hash-t, message identity-t, field censust, normalized
  projection lineage-et és coverage accountingot ellenőriz.
- A jelenlegi `v3/diag/context.py` közvetlen `v3.mcap_reader.McapReader`-t nyitott.
- A jelenlegi `r diag` host dispatch `python -m v3.diag` volt.
- A launcher completion is `v3.diag.registry`-t importált és `*.mcap` inputot
  ajánlott.
- A top-level `r diag` már elkülönült a robot `d -> diag` aliastól; ezt a
  kompatibilitást a refaktor megtartja.

A refaktor ezért új canonical consume oldalt vezet be:

```text
tools/mcap_evidence/reader.py -> tools/diag/
```

A DIAG registry explicit és determinisztikus. A default full analyzer sorrend:

`evidence_health`, `execution_chain`, `safety`, `recovery`, `navigation`,
`localization`, `drive`, `world_model`, `lineage`, `lifecycle`.

A DIAG output szerződésben nincs javaslat/ajánlás csatorna. A registry futáskor
is tiltja a saját outputban a recommendation/suggestion/advice/next_steps/fix
kulcsokat.
