# Mixer Wrapper - Dynamic Reconfigure

## Panoramica

Il nodo `mixer_wrapper` supporta ora la configurazione dinamica dei parametri tramite il sistema di parametri di ROS2.

## Parametri Disponibili

### 1. `serial_port` (string)
- **Default**: `/dev/ttyTHS1`
- **Descrizione**: Porta seriale per comunicare con il mixer
- **Valori comuni**: `/dev/ttyTHS0`, `/dev/ttyTHS1`, `/dev/ttyTHS2`
- **Nota**: Il cambio di questa porta richiede il riavvio del nodo

### 2. `enable_calibration` (bool)
- **Default**: `false`
- **Descrizione**: Abilita la routine di calibrazione
- **Modificabile a runtime**: Sì

### 3. `calibration_duration_sec` (double)
- **Default**: `1.0`
- **Descrizione**: Durata della calibrazione in secondi
- **Modificabile a runtime**: Sì

### 4. `calibration_command_value` (double)
- **Default**: `100.0`
- **Descrizione**: Valore del comando inviato durante la calibrazione
- **Modificabile a runtime**: Sì

## Come Usare

### Metodo 1: Launch file con parametri da YAML

```bash
ros2 launch mixer_wrapper mixer_wrapper.launch.py
```

### Metodo 2: Esecuzione diretta con parametri

```bash
ros2 run mixer_wrapper mixer_com --ros-args --params-file src/mixer_wrapper/config/mixer_params.yaml
```

### Metodo 3: Parametri da linea di comando

```bash
ros2 run mixer_wrapper mixer_com --ros-args \
  -p serial_port:=/dev/ttyTHS0 \
  -p calibration_duration_sec:=2.0 \
  -p calibration_command_value:=150.0
```

## Modificare i Parametri a Runtime

### Usando il comando `ros2 param`

#### Visualizzare tutti i parametri:
```bash
ros2 param list /MIXER_WRAPPER
```

#### Ottenere un valore:
```bash
ros2 param get /MIXER_WRAPPER enable_calibration
```

#### Impostare un valore:
```bash
ros2 param set /MIXER_WRAPPER enable_calibration true
ros2 param set /MIXER_WRAPPER calibration_duration_sec 2.5
ros2 param set /MIXER_WRAPPER calibration_command_value 120.0
```

### Usando rqt_reconfigure (GUI)

```bash
# Installa rqt_reconfigure se non presente
sudo apt install ros-humble-rqt-reconfigure

# Avvia l'interfaccia grafica
ros2 run rqt_reconfigure rqt_reconfigure
```

Seleziona il nodo `/MIXER_WRAPPER` nella lista e modifica i parametri tramite l'interfaccia grafica.

## Avviare la Calibrazione

### Metodo 1: Tramite parametro dinamico
```bash
ros2 param set /MIXER_WRAPPER enable_calibration true
```

### Metodo 2: Tramite topic (retrocompatibilità)
```bash
ros2 topic pub --once /mixer/enable_calibration std_msgs/msg/Bool "{data: true}"
```

## Esempi di Configurazione

### Esempio 1: Cambio porta seriale
Modifica il file `config/mixer_params.yaml`:
```yaml
/**:
  ros__parameters:
    serial_port: "/dev/ttyTHS0"  # Cambiato da ttyTHS1 a ttyTHS0
```

### Esempio 2: Calibrazione più lunga
```bash
ros2 param set /MIXER_WRAPPER calibration_duration_sec 5.0
ros2 param set /MIXER_WRAPPER enable_calibration true
```

### Esempio 3: Comando di calibrazione personalizzato
```bash
ros2 param set /MIXER_WRAPPER calibration_command_value 80.0
ros2 param set /MIXER_WRAPPER enable_calibration true
```

## Build e Installazione

```bash
cd ~/ros2_ws
colcon build --packages-select mixer_wrapper
source install/setup.bash
```

## Troubleshooting

### Problema: Porta seriale non trovata
Se ottieni un errore tipo "Unable to open UART: /dev/ttyTHS1":
1. Verifica che la porta esista: `ls -l /dev/ttyTHS*`
2. Verifica i permessi: `sudo chmod 666 /dev/ttyTHS1`
3. Aggiungi il tuo utente al gruppo dialout: `sudo usermod -a -G dialout $USER`

### Problema: Il parametro serial_port non cambia
Ricorda che il cambio della porta seriale richiede il **riavvio del nodo**.

## Note

- Il sistema mantiene la retrocompatibilità con il topic `/mixer/enable_calibration`
- I parametri possono essere salvati e caricati tramite file YAML
- La configurazione dinamica è thread-safe
