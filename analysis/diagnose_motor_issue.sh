#!/usr/bin/env bash
# =============================================================================
# diagnose_motor_issue.sh
#
# Determina si "solo se mueve el motor[0]" es problema del FIRMWARE o del ENGINE.
#
# Método: aislar cada capa del sistema con stm32monitor.
#
#   FASE 0 — Firmware vivo + baseline (sin engine)
#   FASE 1 — Test mode manual vía stm32monitor (SOLO firmware, sin engine)
#   FASE 2 — Comando normal desde el engine (sin test mode)
#   FASE 3 — Script Lua de caracterización + verificación post-script
#
# Cada fase termina con un veredicto parcial. Al final se imprime el veredicto
# global: FIRMWARE, ENGINE, o AMBOS.
#
# Requiere: stm32monitor en PATH, robot flasheado conectado por ST-Link.
# El robot DEBE estar en el banco (las ruedas van a girar).
#
# Uso:
#   ./diagnose_motor_issue.sh            # modo interactivo guiado
#   ./diagnose_motor_issue.sh --dry-run  # imprime los comandos sin ejecutar
# =============================================================================

set -u

DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=1
fi

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
say()  { printf '\n\033[1;36m%s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m  [OK] %s\033[0m\n' "$*"; }
fail() { printf '\033[1;31m  [FAIL] %s\033[0m\n' "$*"; }
info() { printf '\033[1;33m  [..] %s\033[0m\n' "$*"; }

# Ejecuta un comando stm32monitor (o lo imprime en dry-run)
run() {
    if [[ $DRY_RUN -eq 1 ]]; then
        printf '  \033[90m$ %s\033[0m\n' "$*"
    else
        "$@"
    fi
}

# Pregunta sí/no al operador. Devuelve 0 (sí) o 1 (no).
ask() {
    local prompt="$1"
    local ans
    while true; do
        printf '\033[1;33m  >>> %s [s/N]: \033[0m' "$prompt"
        read -r ans
        case "$ans" in
            s|S|y|Y|si|SI) return 0 ;;
            n|N|no|NO|"")   return 1 ;;
            *) echo "  Respuesta inválida (s/n)" ;;
        esac
    done
}

# ---------------------------------------------------------------------------
# Pre-flight
# ---------------------------------------------------------------------------
say "=== DIAGNÓSTICO: solo se mueve motor[0] ==="

if [[ $DRY_RUN -eq 0 ]]; then
    if ! command -v stm32monitor >/dev/null 2>&1; then
        fail "stm32monitor no está en PATH"
        exit 1
    fi
    ok "stm32monitor encontrado"
fi

info "Asegúrate de que:"
info "  1. El robot está flasheado con el firmware ACTUAL (con el fix de processRadioCommand)"
info "  2. Conectado por ST-Link"
info "  3. En el banco (las ruedas girarán)"
info "  4. El engine está CERRADO (no enviando comandos) para las Fases 0 y 1"
if ! ask "¿Listo para empezar?"; then
    echo "  Abortado."
    exit 1
fi

# ---------------------------------------------------------------------------
# FASE 0 — Firmware vivo + baseline
# ---------------------------------------------------------------------------
say "FASE 0 — Firmware vivo + baseline (engine CERRADO)"

info "Observa si los contadores incrementan (firmware vivo):"
run stm32monitor live dbg_drive_loop dbg_drive_woke
if ask "¿Los contadores dbg_drive_loop / dbg_drive_woke están incrementando?"; then
    ok "Firmware ejecutándose"
else
    fail "El firmware no está corriendo el drive loop — problema de FIRMWARE (task colgada o no flasheado)"
    exit 1
fi

info "Verifica el estado baseline (debe ser test_enable=0, test_motor=0, test_voltage=0):"
run stm32monitor live test_enable test_motor test_voltage
if ask "¿test_enable=0, test_motor=0, test_voltage=0?"; then
    ok "Baseline limpio"
else
    fail "test_enable != 0 SIN engine corriendo — el firmware quedó pegado en test mode."
    fail "Esto apunta a FIRMWARE (o a un engine zombie enviando test_mode)."
    fail "Cierra el engine por completo y repite. Si persiste, es FIRMWARE."
    exit 1
fi

info "Verifica que los 4 motores arrancan habilitados (debe ser 1,1,1,1):"
run stm32monitor live motor[0].enable motor[1].enable motor[2].enable motor[3].enable
if ask "¿Los 4 motores están enable=1?"; then
    ok "Motores habilitados al arranque"
else
    fail "Algún motor arranca deshabilitado — problema de FIRMWARE (init en main.c/drive_task.c)"
    exit 1
fi

# ---------------------------------------------------------------------------
# FASE 1 — Test mode manual vía stm32monitor (SOLO firmware)
# ---------------------------------------------------------------------------
say "FASE 1 — Test mode manual vía stm32monitor (SOLO firmware, engine CERRADO)"
info "Esto prueba el test mode del firmware SIN el engine. Si los 4 motores"
info "giran aquí, el test mode del firmware funciona y el problema está en el engine."

for m in 0 1 2 3; do
    say "  Motor[$m] — test mode manual"
    run stm32monitor native write test_enable 1
    run stm32monitor native write test_motor $m
    run stm32monitor native write test_voltage 300
    info "El motor[$m] debería estar girando ahora (voltage=300)."
    if ask "¿El motor[$m] está girando?"; then
        ok "Motor[$m] responde al test mode del firmware"
    else
        fail "Motor[$m] NO gira en test mode manual."
        fail "→ Si NINGÚN motor gira: problema de FIRMWARE (DAC/DMA/GPIO del test mode)."
        fail "→ Si SOLO motor[0] gira: problema de FIRMWARE (hardware motor[$m] o su canal DAC)."
        run stm32monitor native write test_enable 0
        exit 1
    fi
    run stm32monitor native write test_voltage 0
done

run stm32monitor native write test_enable 0
info "Test mode manual desactivado (test_enable=0)."

info "Verifica que al limpiar test_enable los 4 motores quedan enable=1:"
run stm32monitor live motor[0].enable motor[1].enable motor[2].enable motor[3].enable
if ask "¿Los 4 motores están enable=1 después de test_enable=0?"; then
    ok "El firmware re-habilita los motores al salir de test mode"
else
    fail "Los motores quedan deshabilitados al salir de test mode — problema de FIRMWARE"
    fail "El fix en processRadioCommand() no está funcionando o no está flasheado."
    exit 1
fi

# ---------------------------------------------------------------------------
# FASE 2 — Comando normal desde el engine (sin test mode)
# ---------------------------------------------------------------------------
say "FASE 2 — Comando normal desde el engine (sin test mode)"
info "Ahora ABRE el engine (GUI) y envía un comando de velocidad normal"
info "(ej: vx=0.5 m/s) — SIN ejecutar ningún script Lua."

if ! ask "¿Ya enviaste un comando de velocidad normal desde el engine?"; then
    echo "  Envíalo y vuelve a ejecutar este script desde la FASE 2."
    exit 1
fi

info "Verifica que el engine NO está mandando test mode (debe ser test_enable=0):"
run stm32monitor live test_enable test_motor test_voltage
if ask "¿test_enable=0 con el engine enviando comandos normales?"; then
    ok "El engine no activa test mode con comandos normales"
else
    fail "test_enable=1 con comando normal — problema de ENGINE"
    fail "El engine está serializando test_mode cuando no debería (packet_serializer.rs)."
    exit 1
fi

info "Verifica que los 4 motores reciben voltaje (debe ser >0 en los 4):"
run stm32monitor live motor[0].voltage motor[1].voltage motor[2].voltage motor[3].voltage
if ask "¿Los 4 motores tienen voltage > 0?"; then
    ok "Los 4 motores reciben voltaje con comando normal"
else
    fail "Algún motor tiene voltage=0 con comando normal — problema de FIRMWARE"
    fail "El drive loop no está escribiendo voltaje a ese motor (revisar enable/continue en drive_task.c)."
    exit 1
fi

info "Verifica que los 4 motores giran (measSpeed != 0):"
run stm32monitor live motor[0].measSpeed motor[1].measSpeed motor[2].measSpeed motor[3].measSpeed
if ask "¿Los 4 motores tienen measSpeed != 0?"; then
    ok "Los 4 motores giran con comando normal"
else
    fail "Algún motor tiene voltaje pero no gira — problema de HARDWARE (motor/encoder/DAC canal)"
    exit 1
fi

# ---------------------------------------------------------------------------
# FASE 3 — Script Lua de caracterización + verificación post-script
# ---------------------------------------------------------------------------
say "FASE 3 — Script Lua de caracterización + verificación post-script"
info "Ahora ejecuta el script Lua de caracterización (characterize_wheels.lua)"
info "desde el engine y ESPERA a que termine (imprima 'Complete!')."

if ! ask "¿El script Lua terminó (imprimió 'Complete!')?"; then
    echo "  Ejecútalo y vuelve a este script cuando termine."
    exit 1
fi

info "PUNTO CLAVE — ¿el engine limpió el test mode al terminar?"
info "test_enable debe ser 0 si el engine envió un comando normal al final."
run stm32monitor live test_enable test_motor test_voltage
if ask "¿test_enable=0 después de que el script Lua terminó?"; then
    ok "El engine limpió el test mode al terminar"
    ENGINE_CLEANUP=1
else
    fail "test_enable=1 después de que el script terminó — problema de ENGINE"
    fail "El script Lua (characterize_wheels.lua / characterize.lua) nunca envía un"
    fail "comando de limpieza al terminar: deja test_mode activo en el robot."
    ENGINE_CLEANUP=0
fi

info "Verifica el estado de los motores AHORA (después del script):"
run stm32monitor live motor[0].enable motor[1].enable motor[2].enable motor[3].enable
if ask "¿Los 4 motores están enable=1?"; then
    ok "Motores habilitados después del script"
    MOTORS_OK=1
else
    fail "Motores deshabilitados después del script"
    MOTORS_OK=0
fi

# ---------------------------------------------------------------------------
# VEREDICTO GLOBAL
# ---------------------------------------------------------------------------
say "=== VEREDICTO GLOBAL ==="

if [[ ${ENGINE_CLEANUP:-0} -eq 0 && ${MOTORS_OK:-0} -eq 0 ]]; then
    fail "PROBLEMA DE ENGINE (principal): el script Lua no limpia test_mode al terminar."
    fail "PROBLEMA DE FIRMWARE (secundario): el fix de processRadioCommand() no re-habilita"
    fail "los motores, o no está flasheado."
    echo ""
    echo "  Acciones:"
    echo "    1. Fix engine: characterize_wheels.lua / characterize.lua deben enviar"
    echo "       un comando normal (o test_motor_step con voltage 0) al terminar."
    echo "    2. Fix firmware: verificar que el fix en processRadioCommand() está"
    echo "       flasheado (FASE 1 lo confirma: si los 4 motores quedan enable=1"
    echo "       al salir de test mode manual, el fix SÍ funciona)."
elif [[ ${ENGINE_CLEANUP:-0} -eq 0 ]]; then
    fail "PROBLEMA DE ENGINE: el script Lua no limpia test_mode al terminar."
    echo ""
    echo "  Acción: fix en characterize_wheels.lua / characterize.lua — enviar"
    echo "  comando de limpieza (test_motor_step con voltage 0 o comando normal)"
    echo "  cuando la caracterización termina."
elif [[ ${MOTORS_OK:-0} -eq 0 ]]; then
    fail "PROBLEMA DE FIRMWARE: test_enable se limpió pero los motores quedaron"
    fail "deshabilitados — el fix de processRadioCommand() no está funcionando"
    fail "o no está flasheado."
    echo ""
    echo "  Acción: verificar que el fix está en drive_task.c y reflashear."
else
    ok "TODO FUNCIONA: firmware y engine limpian el test mode correctamente."
    echo ""
    echo "  Si el problema persiste, es de HARDWARE (motor/encoder/DAC) o el"
    echo "  engine no está enviando los comandos que crees (revisar radio)."
fi

echo ""
echo "Diagnóstico completo."