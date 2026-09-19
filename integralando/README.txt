R2B4 LLM-P0/P1 upgrade

Adds only new files. No existing V3 production file is modified by this package.

Implemented:
- bounded asynchronous conversation ingress
- append-only 0600 NDJSON conversation journal
- fresh RobotInterface-derived robot context before every LLM turn
- versioned system prompt under conf/voice_llm_system.md
- Groq structured-output LLM client, default openai/gpt-oss-20b
- immutable UserTextTurn / LLMDecision / RobotAction host contracts
- high-level action allowlist + parameter validation
- SHADOW intent mode: no LLM action is executed
- conversation.submit_text / conversation.status / conversation.last_turn through a composed RobotInterface
- motor-free CLI for check and live text-only LLM testing

Not yet in this slice:
- STT post-wake wiring into conversation.submit_text
- TTS output
- execution of robot intents
- v3.person / v3.mission / v3.world read capabilities

Install:
1. Extract ZIP.
2. Copy extracted directory contents to /home/alba/project_r2b4/integralando/
3. Run from that directory: python3 installer.py
