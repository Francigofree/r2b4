# Source-first report

A friss live faultokban az L6 `ASYNC_L6_PLAN_STALE` úgy jelent meg, hogy a
`PlannerInput` üres maradt: a korábbi accepted plan öregedett ki, miközben a
replacement pending volt. A post-tick dispatch már megszüntette az előző
egy-tickes submit késést; a fennmaradó probléma continuity-szemantika.

Ez a refaktor nem engedi stale plan használatát. Ha replacement még szabályosan
pending, L6 IDLE `PLANNER_STALE_HOLD` tervet ad, amit L7 stop objective-re visz.
Ha fresh completion érkezik, a mission folytatódik. Transport deadline, worker
failure, source mismatch vagy már elkészült stale result továbbra is FAULT.

A CPU/GIL/affinity fejlesztés szünetel: a külön CPU diag tool perturbálja az
RPi5 futását. Az új evidence kizárólag kicsi scalar capability state/counter.
