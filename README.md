# Expert — Firmware RTOS

## Descrição

Controlador de mouse gestual usando MPU-6050 e FreeRTOS no Raspberry Pi Pico 2 (RP2350).  
Os dados do IMU são lidos pela `mpu_task`, processados pelo algoritmo AHRS (Fusion) na `fusion_task`, e enviados via USB-CDC pela `uart_task`. O LED RGB é controlado pela `pwm_task` com suavização EMA.

Pinos de debug (GPIO toggled para medição no Saleae):

| Task | GPIO |
|---|---|
| mpu_task | 15 |
| fusion_task | 11 |
| uart_task | 12 |
| pwm_task | 13 |

---

## Métricas de RTOS

> Medidas com a extensão **MetricasSaleae** no Saleae Logic 2.  
> WCET = maior largura de pulso HIGH observada.  
> Jitter = Período máx − Período mín entre bordas de subida consecutivas.  
> Deadline Miss Rate = % de execuções com WCET > período médio medido.  
> Stack Usage = medido via `uxTaskGetStackHighWaterMark` pela UART serial.

### Tabela — Single Core

| Métrica / Task     | mpu_task  | fusion_task | uart_task | pwm_task  |
|--------------------|-----------|-------------|-----------|-----------|
| WCET (µs)          | 61,70     | 11,10       | 17,00     | 2,80      |
| Jitter (µs)        | 0,10      | 3,10        | 7,50      | 7,55      |
| Deadline Miss Rate | 0%        | 0%          | 0%        | 0%        |
| Stack Usage        | 0% | 1% | 1% | 2% |

### Tabela — SMP (2 cores)

| Métrica / Task     | mpu_task  | fusion_task | uart_task | pwm_task  |
|--------------------|-----------|-------------|-----------|-----------|
| WCET (µs)          | 65,40     | 32,20       | 77,50     | 9,40      |
| Jitter (µs)        | 1,70      | 7,60        | 16,30     | 21,70     |
| Deadline Miss Rate | 0%        | 0%          | 0%        | 0%        |
| Stack Usage        | 0% | 1% | 1% | 2% |

---

## Análise — Single Core vs SMP

- **mpu_task**: WCET praticamente igual nos dois modos. O tempo é dominado pela leitura I2C (hardware), independente do número de cores.
- **fusion_task**: WCET quase triplicou em SMP (11 → 32 µs). Sem afinidade de core fixa, a task pode rodar num core diferente da `mpu_task`, adicionando overhead de sincronização inter-core ao acessar a fila `xQueueMPU`.
- **uart_task**: Maior impacto em SMP (17 → 77 µs). O USB-CDC usa spinlocks internos do Pico SDK que ficam mais lentos com dois cores ativos competindo pelo barramento.
- **pwm_task**: Jitter disparou em SMP (7 → 21 µs). O scheduler SMP migra a task entre cores livremente, causando variação no tempo de execução.
- **DMR = 0%** em todos os casos: nenhuma task perdeu deadline em single core ou SMP.
