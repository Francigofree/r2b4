async def run(robot):
    scenario = await robot.read("simulation.scenario")
    return await robot.call(
        "v3.command.move_relative",
        forward_m=scenario["goal"]["x_m"],
        max_v_mps=0.20,
    )
