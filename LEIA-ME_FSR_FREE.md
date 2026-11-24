# OpenExo — ramo FSR-free (coleta de dados + controle sem FSR)

Base: OpenExo `main`, commit `523852a` (2026-06-11).
Compilado e ligado sem erros (GCC 13, núcleos Teensy e Arduino mbed do GitHub) para:
Teensy 4.1 nos três modos de fase e Nano 33 BLE com LSM9DS1 e com BMI270.
**Ainda não foi testado no hardware.**

## O que faz

| Placa | O que mudou |
|---|---|
| Nano 33 BLE | Lê a IMU da placa (acelerômetro + giroscópio) a 100 Hz e manda à Teensy pelo UART já existente (comando novo `0x1C`). |
| Teensy 4.1 | Filtro complementar → inclinação da pelve. Dois estimadores de fase (retrato de fase, Choi 2023) rodando sempre: só encoder e encoder + IMU. Gravação de tudo no cartão SD a 500 Hz. Opção de usar o estimador no lugar do FSR. |

Grava, por lado: FSR (calcanhar, dedo, eventos, % marcha), posição/velocidade/corrente do motor, comando de torque, leitura de torque, bytes brutos 6 e 7 da resposta CAN, saídas dos dois estimadores, flag de proteção e tempo de execução do estimador. Globais: IMU bruta, inclinação da pelve, idade do último pacote da IMU, estado do exo, LED de sincronismo.

## Escolher o modo pelo app (sem regravar)

Controlador de quadril novo: **`fcNoFSR`**. É o Franks–Collins original, com os mesmos 15 parâmetros, mais um 16º, **`phase_src`**: 1 = só encoder; 2 = encoder + IMU (C2).
- C1: escolha `franksCollinsHip` no app (fase pelos FSRs).
- C2: escolha `fcNoFSR` no app (fase pelo estimador).
- Copie `SDCard/hipControllers/fcNoFSR.csv` para a pasta `hipControllers/` do cartão. Sem esse arquivo, o controlador não aparece no app.
- Para ele ser o padrão ao ligar: `hipDefaultController = fcNoFSR` no `config.ini`.
- Os dois estimadores e os FSRs são gravados no log com qualquer controlador; a coluna `phase_source` mostra qual fase estava em uso.

## Antes de gravar: conferir `ExoCode/src/FsrFreeConfig.h`

1. **`PHASE_SOURCE`** — deixe em `PHASE_SOURCE_FSR` (padrão). Só mude se quiser forçar o estimador em *todos* os controladores.
2. **`NANO33_IMU_BMI270`** — descomente se o Nano for Rev2 (BMI270). Original = LSM9DS1.
3. **Sinais (CHECK)** — `HIP_FLEXION_SIGN_LEFT/RIGHT`: flexão positiva nos dois lados.
4. **Eixos do Nano na caixa (CHECK)** — `PELVIS_AXIS_*` e `PELVIS_SIGN_*`. Parado em pé, o eixo VERTICAL deve ler ≈ +1 g; inclinando o tronco para frente, `pelvis_tilt_deg` deve aumentar.
5. **`PP_PHASE_OFFSET_PCT`** (17 %, valor do Choi) e **`PP_TOE_OFF_PCT`** (60 %) — valores iniciais; identificar com os seus dados de C0/C1.

## Gravação

Arduino IDE 2 + Teensyduino; pacote "Arduino Mbed OS Nano Boards"; bibliotecas do OpenExo de sempre + **Arduino_LSM9DS1** (ou **Arduino_BMI270_BMM150**).

1. Abra `ExoCode/ExoCode.ino`.
2. Placa **Arduino Nano 33 BLE** → Upload.
3. Placa **Teensy 4.1** → Upload.
4. Cartão SD na Teensy com o `config.ini` de sempre.

O mesmo sketch serve às duas placas, como no OpenExo original.

## Teste de bancada sugerido (antes de pessoas)

1. Exo na mesa, torque zero, iniciar trial no app → deve aparecer `LOG000.BIN` no SD.
2. Mover a coxa à mão: `L_pp_enc_in_deg` aumenta na flexão? (senão inverta o sinal)
3. Inclinar a caixa para frente: `pelvis_tilt_deg` aumenta? `imu_ok = 1`? `imu_age_us` < 20 000?
4. `R_est_exec_us` e `loop_dt_us` (deve ficar ≈ 2000 µs).
5. Parar o trial no app antes de desligar (fecha o arquivo).

## Ler os dados

```
python tools/oexlog_to_csv.py LOG000.BIN
```
Gera `LOG000.csv` (primeira linha = configuração usada na gravação).

## Proteções (modo sem FSR)

Torque do quadril = 0 enquanto o estimador não for válido: menos de 2 passadas desde o início, amplitude < 5°, passada fora de 600–2000 ms, mais de 1,5 passada sem máximo de flexão, ou IMU sem dados há 50 ms (modo encoder + IMU).

## Pontos a conferir

- **Bytes 6 e 7 da resposta CAN** são gravados brutos (`can_b6`, `can_b7`). Em alguns firmwares AK são temperatura e código de erro; confirmar no manual do seu modelo CubeMars antes de interpretar.
- **O controlador Franks–Collins deste commit usa `percent_stance`** (via `_get_percent_gait`), não `percent_gait`, junto com `expected_step_duration`. No modo sem FSR, o estimador fornece os dois; a fase de apoio é aproximada por `PP_TOE_OFF_PCT`.
- Ao parar de andar, a assistência só é zerada depois de ~1,5 passada sem novo máximo de flexão.
- `SD.begin()` agora é chamado uma vez só (`SdOnce.h`), para não reinicializar o cartão com o arquivo de log aberto.

## Arquivos

Novos: `SDCard/hipControllers/fcNoFSR.csv`, `FsrFreeConfig.h`, `PhasePortrait.h`, `PelvisImu.h/.cpp`, `DataLoggerSD.h/.cpp`, `SdOnce.h`, `tools/oexlog_to_csv.py`.
Alterados: `ExoCode.ino`, `Exo.cpp`, `Side.h/.cpp`, `SideData.h`, `JointData.h`, `Joint.cpp`, `MotorData.h`, `Motor.cpp`, `uart_commands.h`, `ParamsFromSD.cpp/.h`, `ListCtrlParams.cpp`, `ParseIni.cpp/.h`, `ControllerData.h/.cpp`, `Joint.h`, `SDCard/config.ini` (comentário).
Diferenças completas: `fsr_free.patch`.
