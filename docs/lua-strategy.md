# Estrategia Lua

## API de Control (expuesta por el engine a Lua)

```lua
-- Movimiento directo
send_velocity(id, team, vx, vy, omega)

-- Movimiento con path planning + bangbang
move_to(id, team, {x, y})
move_direct(id, team, {x, y})

-- Movimiento con corrección PID
motion(id, team, {x, y}, kp_x?, ki_x?, kp_y?, ki_y?)

-- Rotación
face_to(id, team, {x, y}, kp?, ki?, kd?)

-- Kicker
kickx(id, team)
kickz(id, team)
dribbler(id, team, speed)  -- speed: 0-10

-- PID management
reset_pids()
reset_pids_for(id)

-- Trayectoria personalizada
bangbang_trajectory(id, team, v_max, a_max, path_table)
```

## API de Estado

```lua
get_robot_state(id, team)      -- -> {x, y, vel_x, vel_y, orientation, omega, active}
get_ball_state()               -- -> {x, y, vel_x, vel_y}
get_blue_team_state()          -- -> table of RobotState
get_yellow_team_state()        -- -> table of RobotState
get_ref_message()              -- -> "HALT", "STOP", "START", etc.
```

## API de GUI Overlay

```lua
draw_point(x, y, color?)
draw_line({p1, p2, ...}, color?)
draw_text(x, y, text, color?)
highlight_robot(id, team)
```

## API de Simulación (grSim)

```lua
grsim.teleport_robot(id, team, x, y, dir)
grsim.teleport_ball(x, y)
```

## Scripts de Estrategia

### `lua/main.lua` — Formation Demo
- 6 robots en fila (X: -0.80 a 0.70, step 0.3)
- Máquina de estados por robot:
  - State 0: face_to → Y_B = 0.5
  - State 1: move_to → Y_B (manteniendo orientación)
  - State 2: face_to → Y_A = -0.5
  - State 3: move_to → Y_A
- Usa `reset_pids_for(robot.id)` en cada transición de estado

### `lua/skills/`

| Skill | Descripción |
|---|---|
| `move.lua` | `move_to()` wrapper con tolerancia de distancia |
| `go_to_ball.lua` | Dos fases: stage a 0.14m → close a 0.05m con dribbler |
| `kick.lua` | Flat kick simple |
| `kick_to_point.lua` | Alineación detrás de pelota, verifica kick line, approach + kick |
| `move_ball.lua` | Dribbling: go_to_ball + dribbler + move_direct hacia target |

### `lua/tactics/`

| Táctica | Descripción |
|---|---|
| `receive_pass.lua` (active) | Posiciona robot entre pelota y sí mismo a 0.15m, facing ball |
| `position_for_pass.lua` (non-active) | Grid-search scoring: mejor posición dentro de región para recibir pase (penaliza <1m de pelota, prefiere 0.8m) |

### `lua/utils/utils.lua`
- Ball possession: distancia < 0.12m y ángulo < 0.35 rad

## Flujo de Llamadas

```
process() (called @ ~60 FPS by engine)
  │
  ├── get_robot_state(id, team)
  ├── get_ball_state()
  │
  ├── face_to(id, team, target)
  │     └── motion.mod.rs: face_pid[robot.id].compute(error)
  │
  ├── move_to(id, team, target)
  │     └── motion.mod.rs: planner.get_path() + bangbang.compute()
  │
  ├── motion(id, team, target, kp_x, ki_x, kp_y, ki_y)
  │     └── motion.mod.rs: planner + bangbang + motion_pid_x/y[robot.id].compute()
  │
  ├── kickx/kickz/dribbler
  │     └── radio.rs: add_kicker_command()
  │
  └── draw_point/line/text/highlight
        └── gui overlay via channel
```

## Notas de Diseño

- Cada robot tiene su propio estado PID en el engine (HashMap keyed por robot ID)
- `reset_pids()` se llama automáticamente al recargar script
- `reset_pids_for(id)` permite reset selectivo al cambiar de objetivo
- Los comandos de motion se mergean en radio.rs: send_velocity + face_to se combinan (vx/vy se suman, angular se overridea si ≠ 0)
- La estrategia corre en el engine loop (~60 FPS), no en tiempo real
