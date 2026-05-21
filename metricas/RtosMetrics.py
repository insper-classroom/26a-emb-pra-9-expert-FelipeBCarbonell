# =============================================================================
#  RTOS Metrics — Saleae Logic 2 Measurement Extension
#  Autor: FelipeBCarbonell
#
#  Como usar:
#    1. Instale a extensão no Logic 2:
#       Extensions → Load Existing Extension → selecione a pasta "metricas"
#    2. Conecte os pinos de debug ao Saleae:
#         GPIO 10 → Channel 0  (mpu_task)
#         GPIO 11 → Channel 1  (fusion_task)
#         GPIO 12 → Channel 2  (uart_task)
#         GPIO 13 → Channel 3  (led_task)
#    3. Grave o sinal e selecione um intervalo de tempo em qualquer canal digital.
#    4. No painel "Measurements" à direita, procure "RTOS Metrics" e expanda.
#
#  Métricas calculadas (por canal selecionado):
#    • WCET (µs)              — maior largura de pulso observada
#    • WCET min (µs)          — menor largura de pulso observada
#    • WCET avg (µs)          — largura média de pulso
#    • Jitter (µs)            — Pmax − Pmin (variação do período entre ativações)
#    • Period max (µs)        — maior período entre bordas de subida
#    • Period min (µs)        — menor período entre bordas de subida
#    • Period avg (µs)        — período médio (≈ 1/frequência real)
#    • f max (Hz)             — frequência máxima possível = 1/WCET
#    • Deadline Miss Rate (%) — % de pulsos com duração > período configurado
#    • Executions             — número de pulsos completos no intervalo
#
#  NOTA — Stack Usage:
#    Não é possível medir Stack Usage pelo osciloscópio/Saleae.
#    Para medir, descomente stack_monitor_task no main.c e leia
#    a saída pelo terminal serial (PuTTY, minicom, etc.).
# =============================================================================

from saleae.range_measurements import DigitalMeasurer
from saleae.analyzers import NumberSetting


class RtosMeasurer(DigitalMeasurer):
    """
    Mede métricas de tempo real (WCET, Jitter, Deadline Miss Rate)
    a partir de um pino de debug GPIO toggled por uma FreeRTOS task.

    Configuração:
      task_period_ms — período nominal da task em milissegundos.
                       Usado para calcular o Deadline Miss Rate.
                       Exemplos:
                         mpu_task    → 10 ms
                         fusion_task → 10 ms
                         uart_task   → (event-driven, use 15 ms)
                         led_task    → (event-driven, use 20 ms)
    """

    # Período configurado da task (ms) — altere conforme a task que está medindo
    task_period_ms = NumberSetting(min_count=0.1, max_count=100000.0)

    supported_measurements = [
        'WCET (us)',
        'WCET min (us)',
        'WCET avg (us)',
        'Jitter (us)',
        'Period max (us)',
        'Period min (us)',
        'Period avg (us)',
        'f max (Hz)',
        'Deadline Miss Rate (%)',
        'Executions',
    ]

    def measure(self, data):
        """
        Processa as transições digitais do intervalo selecionado.

        Definições:
          - Pulso (WCET):  tempo entre borda de subida e a próxima borda de descida
          - Período:       tempo entre duas bordas de subida consecutivas
          - Deadline miss: pulso com duração > task_period_ms configurado
        """

        pulse_widths = []   # segundos — largura de cada pulso HIGH
        periods = []        # segundos — período entre ativações consecutivas

        t_rise = None       # timestamp da última borda de subida
        t_last_rise = None  # timestamp da borda de subida anterior

        # Itera sobre as transições do intervalo selecionado.
        # Cada item é uma tupla (time_seconds: float, state: bool).
        for t, state in data:
            if state:
                # ── Borda de SUBIDA ──────────────────────────────────────────
                if t_last_rise is not None:
                    periods.append(t - t_last_rise)
                t_last_rise = t
                t_rise = t
            else:
                # ── Borda de DESCIDA ─────────────────────────────────────────
                if t_rise is not None:
                    pulse_widths.append(t - t_rise)
                    t_rise = None

        # ── Período configurado da task (convertido para segundos) ────────────
        period_s = float(self.task_period_ms) * 1e-3

        results = {}

        # ── Métricas de WCET ─────────────────────────────────────────────────
        if pulse_widths:
            wcet_max = max(pulse_widths)
            wcet_min = min(pulse_widths)
            wcet_avg = sum(pulse_widths) / len(pulse_widths)

            results['WCET (us)']     = wcet_max * 1e6
            results['WCET min (us)'] = wcet_min * 1e6
            results['WCET avg (us)'] = wcet_avg * 1e6
            results['f max (Hz)']    = 1.0 / wcet_max if wcet_max > 0 else 0.0
            results['Executions']    = float(len(pulse_widths))

            # ── Deadline Miss Rate ────────────────────────────────────────────
            # Um "deadline miss" ocorre quando o tempo de execução da task
            # ultrapassa o período configurado (não há tempo para completar
            # antes da próxima ativação).
            misses = sum(1 for pw in pulse_widths if pw > period_s)
            results['Deadline Miss Rate (%)'] = (misses / len(pulse_widths)) * 100.0

        # ── Métricas de Jitter ───────────────────────────────────────────────
        # Jitter = Pmax − Pmin (variação do período entre ativações).
        # Requer pelo menos 2 períodos observados (3 bordas de subida).
        if len(periods) >= 2:
            p_max = max(periods)
            p_min = min(periods)
            p_avg = sum(periods) / len(periods)

            results['Jitter (us)']     = (p_max - p_min) * 1e6
            results['Period max (us)'] = p_max * 1e6
            results['Period min (us)'] = p_min * 1e6
            results['Period avg (us)'] = p_avg * 1e6

        return results
