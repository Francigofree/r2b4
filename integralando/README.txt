R2B4 ChatGPT/OpenAI primary LLM upgrade
========================================

Target source:
  Francigofree/r2b4
  main @ ceaf835a69d8eac607919f2d42e7d9a5015dba0b
  2026-10-02 18:14:42 UTC

What changes:
- OpenAI Responses API provider is added without a new Python package dependency.
- Default LLM provider becomes OpenAI/ChatGPT.
- Default OpenAI model is gpt-5.6 (GPT-5.6 Sol alias).
- Existing Gemini and Groq providers remain selectable fallbacks.
- Root `r "..."` plain prompt path follows the selected provider.
- Voice/conversation/AgentCore uses the same provider selection.
- Existing R2B4 AgentCore tool broker and canonical RobotInterface/V3 safety path stay unchanged.
- System behavior contract is updated so source and authority agree.
- Local TTS stays separate; Gemini TTS can still use GEMINI_API_KEY if configured.

Install:
  unzip r2b4_chatgpt_primary_upgrade_20261002.zip
  cd r2b4_chatgpt_primary_upgrade_20261002
  python3 install.py /home/alba/project_r2b4

Required secret:
  Add OPENAI_API_KEY=<your OpenAI API key> to:
    /home/alba/project_r2b4/conf/.wake.env
  Keep the file mode 600:
    chmod 600 /home/alba/project_r2b4/conf/.wake.env

The installer switches:
  R2B4_LLM_PROVIDER=openai
  R2B4_LLM_MODEL=gpt-5.6

Minimal validation after install:
  cd /home/alba/project_r2b4
  python3 -m r2b4_voice.voice_service --check
  python3 -m pytest -q tests/core/test_openai_responses_transport.py tests/core/test_openai_primary_provider.py

Then restart voice if you use the user service:
  systemctl --user restart r2b4-wake.service

Simple fallback examples:
  R2B4_LLM_PROVIDER=gemini
  R2B4_LLM_MODEL=gemini-2.5-flash

or:
  R2B4_LLM_PROVIDER=groq
  R2B4_LLM_MODEL=openai/gpt-oss-20b

Notes:
- ChatGPT Plus and OpenAI API billing/keys are separate products.
- The installer does not start V3 or move the robot.
- It creates a backup under .upgrade_backups/ before writing.
