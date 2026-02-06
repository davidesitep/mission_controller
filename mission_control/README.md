# USV Mission Control System - Installazione

## Prerequisiti
- ROS Noetic
- Python 3.8+
- Pacchetti: `sbg_driver`, `move_base`, `robot_localization`

## Installazione

### 1. Copia i file nel workspace ROS
```bash
cd ~/catkin_ws/src
git clone <tuo_repo> usv_mission_control
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Installa dipendenze Python
```bash
pip3 install gpxpy --break-system-packages
```

### 3. Integra nel launch file esistente
Aggiungi al vostro `main.launch`:
```xml
<include file="$(find usv_mission_control)/launch/mission_control.launch"/>
```

### 4. (OPZIONALE) Installa Supervisord per robustezza
```bash
sudo apt update
sudo apt install supervisor

# Copia configurazione
sudo cp config/supervisor_usv.conf /etc/supervisor/conf.d/

# Attiva
sudo supervisorctl reread
sudo supervisorctl update

# Verifica
sudo supervisorctl status
```

## Test

### Verifica nodi attivi
```bash
rosnode list
# Dovresti vedere:
# /mission_controller
# /mission_watchdog
# /usv_diagnostic
# /mission_ext_interface
```

### Test watchdog
Killa il mission controller:
```bash
rosnode kill /mission_controller
```
Il watchdog dovrebbe rilevarlo entro 5s e loggare l'errore.

## Configurazione

### Parametri modificabili
- `mission_directory`: Path dove cercare file missione GPX
- `drone_id`: ID univoco del veicolo

### Topic pubblicati
- `/mission_controller/heartbeat`: Heartbeat (1 Hz)
- `/cmd_vel`: Comandi velocità
- `/usv_status`: Stato FSM (Int32)

### Topic sottoscritti
- `/move_base/feedback`: Feedback navigazione
- `/diagnostic/status`: Stato sensori
- `/teleop/cmd_vel`: Comandi remoti

## Troubleshooting

### Mission controller non parte
```bash
# Verifica errori
rosnode info /mission_controller
# Vedi log
tail -f ~/.ros/log/latest/mission_controller*.log
```

### Watchdog non rileva crash
Verifica pubblicazione heartbeat:
```bash
rostopic hz /mission_controller/heartbeat
# Dovrebbe essere ~1 Hz
```
```

---

## 🎁 **DELIVERABLE FINALE AL CLIENTE**

### **Opzione MINIMA (solo ROS)**
```
Cosa consegni:
✅ Codice Python (4 file)
✅ Launch file da includere nel loro
✅ README con istruzioni

Il cliente fa:
1. Copia file in catkin_ws/src
2. catkin_make
3. Aggiunge <include> al loro launch file
4. Testa
```

**Pro**: Minimo impatto sul loro sistema  
**Contro**: Nessun watchdog esterno (solo quello ROS interno)

---

### **Opzione CONSIGLIATA (ROS + Supervisord)**
```
Cosa consegni:
✅ Codice Python (4 file)
✅ Launch file
✅ Configurazione Supervisord
✅ README dettagliato

Il cliente fa:
1. Copia file in catkin_ws/src
2. catkin_make
3. Installa supervisor (una riga: apt install supervisor)
4. Copia config Supervisord
5. supervisorctl update
```

**Pro**: Robustezza massima, roscore monitorato  
**Contro**: Devono installare Supervisord (5 minuti)

---

### **Opzione ENTERPRISE (tutto automatizzato)**
```
Cosa consegni:
✅ Codice Python
✅ Launch file
✅ Script install.sh che fa tutto automaticamente
✅ Configurazione Supervisord
✅ Documentazione completa

Il cliente fa:
1. ./install.sh
2. Fine