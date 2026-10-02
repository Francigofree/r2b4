# R2B4 Agent Core

`R2B4_AGENT_SYSTEM_V1` introduces a small host-side reasoning/tool loop. It is
not a robot authority and does not add an L13 or alternate control path.

```text
human text
   |
   +-- exact STOP --> canonical v3.command.stop
   |
   `-- everything else --> AgentCore --> configured LLM
                                  |
                                  +-- final answer
                                  +-- source/docs/config/EVI/DIAG tools
                                  +-- policy-gated config.patch
                                  +-- canonical RobotAction proposal
                                  `-- ER2 delegation
```

Robot actions remain proposals until the existing fresh-state validator and
`RobotInterface` path accept them. ER2 still reaches motion only through
`ExternalRobotGateway -> RobotInterface -> V3`. Config writes are limited to a
conservative tuning allowlist and run as stop/validate/atomic-write/restart
transactions with rollback.

The Agent Core itself uses only Python standard-library primitives. Domain
systems remain authoritative in their existing modules; the core only composes
explicit adapters. New tuning tools should be registered only after they have a
canonical typed API; there is no dynamic plugin discovery.
